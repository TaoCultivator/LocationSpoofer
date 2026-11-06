@file:Suppress(
    "UNUSED_PARAMETER",
    "UNUSED_VARIABLE",
    "UNNECESSARY_NOT_NULL_ASSERTION",
    "DEPRECATION",
    "NAME_SHADOWING",
    "FunctionName",
    "PrivatePropertyName",
    "SpellCheckingInspection",
    "RedundantUnitReturnType",
    "RemoveRedundantQualifierName",
    "OPT_IN_USAGE",
    "unused",
    "UnusedImport"
)

package com.vincenthzr.locationspoofer.xposed.hooks

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import com.vincenthzr.locationspoofer.utils.GaitTemplate
import com.vincenthzr.locationspoofer.xposed.LocationHooker
import com.vincenthzr.locationspoofer.xposed.utils.RouteEngine
import com.vincenthzr.locationspoofer.xposed.utils.XposedHelpers
import org.json.JSONObject
import java.lang.reflect.Constructor
import java.lang.reflect.Field
import java.util.Random
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.CopyOnWriteArrayList
import kotlin.math.floor

/**
 * 计步 / 加速度计伪造。
 *
 * 这一版修的核心问题：
 *
 * 1. **速度以"位置推出来的速度"为准**，不再直接读配置里的 `speed_m_s`。
 *    路线到达终点、暂停时配置里的速度不会归零，只有 [RouteEngine.calculateCurrentPosition] 的
 *    结果会变成 0；用错就会出现"人已经停在终点，步数还在涨"。
 *
 * 2. **`start_timestamp` 变化时补时间，而不是重新开始**。
 *    摇杆模式每秒（转向/变速时 250ms）重写一次 `start_timestamp`。步数如果按"从 StartTimestamp
 *    起累计"算，每重写一次就丢一段，只有纯计步器监听的 App 会几乎不涨步数。这里在重设基准时
 *    把 `[上次计算时刻, 新 StartTimestamp]` 按当前步频补进去，超过 [MAX_STEP_GAP_MS] 的间隔
 *    （暂停 / 进程空闲）不补，避免重新走动时凭空多出几百步。
 *
 * 3. **状态只在 [stepLock] 内改写**。步数会被传感器分发线程、Listener 投递线程、主线程
 *    （每秒主动推送）同时读写；不锁会出现旧值覆盖新值、累计步数倒退。真实计步器是单调的，
 *    倒退是最容易被判异常的信号。
 *
 * 4. **步进检测器按步频补发**，不再固定 1Hz。检测器语义是"每走一步报一次"，固定 1Hz 会和
 *    `TYPE_STEP_COUNTER` 的累计值自相矛盾（走路时 counter 约 2~2.75 步/秒）。
 */
object SensorStepHooker {

    data class CapturedSensorListener(
        val listener: Any,
        val sensor: Sensor?,
        val handler: Handler?
    )

    val capturedListeners = CopyOnWriteArrayList<CapturedSensorListener>()
    private val hookedListenerClasses = ConcurrentHashMap<Class<*>, Boolean>()

    // 真实计步器自开机以来单调递增。这里只作为进程首次介入时的种子，之后由 stepBase 接续。
    private const val INITIAL_BOOT_STEPS = 2350.0

    /** 低于这个速度视为静止：不推进步数，也不主动推事件 */
    private const val STATIONARY_SPEED = 0.05

    /** 低于这个速度视为"没有真实运动"：不合成加速度、不推步进检测器 */
    private const val MOTION_SPEED = 0.1

    /** 重设 start_timestamp 时最多向前补多少毫秒的步数；暂停 / 进程空闲的大间隔不补 */
    private const val MAX_STEP_GAP_MS = 2_500L

    /** 单次 tick 最多补发几步检测器事件，避免进程挂起后一次性爆发 */
    private const val MAX_DETECTOR_BURST = 6

    /** 保护 stepBase / lastStepsFloat / lastStepInitTime / lastStepComputedAt / lastDetectorStep */
    private val stepLock = Any()

    @Volatile private var stepBase: Double = INITIAL_BOOT_STEPS
    @Volatile private var lastStepsFloat: Double = INITIAL_BOOT_STEPS
    @Volatile private var lastStepInitTime: Long = 0L

    /** 上一次真正算出步数的时刻，用于重设 start_timestamp 时补时间 */
    @Volatile private var lastStepComputedAt: Long = 0L

    /** 已经推给监听器的检测器计数（与 floor(total) 对齐） */
    @Volatile private var lastDetectorStep: Double = 0.0

    private val noiseRng = Random()
    @Volatile private var cachedTemplateSource: String? = null
    @Volatile private var cachedTemplate: GaitTemplate? = null

    // 缓存虚拟 Sensor 实例
    private var mockStepCounterSensor: Sensor? = null
    private var mockStepDetectorSensor: Sensor? = null
    private var mockAccelerometerSensor: Sensor? = null

    // 跟踪 handle 到 sensor type 的映射
    private val handleToTypeMap = ConcurrentHashMap<Int, Int>()

    /** 每秒主动推送时复用同一个 Handler，避免每个 listener 每次都 new */
    private val mainLooperHandler: Handler? by lazy {
        try {
            Handler(Looper.getMainLooper())
        } catch (_: Throwable) {
            null
        }
    }

    fun hookSensorStepSimulation(classLoader: ClassLoader) {
        val targetClasses = listOf(
            "android.hardware.SensorManager",
            "android.hardware.SystemSensorManager"
        )

        for (className in targetClasses) {
            val managerClass = XposedHelpers.findClassIfExists(className, classLoader) ?: continue

            // 1. Hook getDefaultSensor: 当设备无物理计步传感器时注入虚拟 Sensor
            try {
                XposedHelpers.hookAllMethods(managerClass, "getDefaultSensor") { chain, _ ->
                    val result = chain.proceed(chain.args.toTypedArray())
                    val type = (chain.args.firstOrNull() as? Int) ?: return@hookAllMethods result
                    if (result == null && (type == Sensor.TYPE_STEP_COUNTER || type == Sensor.TYPE_STEP_DETECTOR)) {
                        return@hookAllMethods getOrCreateMockSensor(type, classLoader)
                    }
                    return@hookAllMethods result
                }
            } catch (_: Throwable) {}

            // 2. Hook getSensorList
            try {
                XposedHelpers.hookAllMethods(managerClass, "getSensorList") { chain, _ ->
                    val result = chain.proceed(chain.args.toTypedArray())
                    val type = (chain.args.firstOrNull() as? Int) ?: return@hookAllMethods result
                    if (type == Sensor.TYPE_STEP_COUNTER || type == Sensor.TYPE_STEP_DETECTOR) {
                        if (result is List<*> && result.isEmpty()) {
                            val mock = getOrCreateMockSensor(type, classLoader)
                            if (mock != null) return@hookAllMethods listOf(mock)
                        }
                    }
                    return@hookAllMethods result
                }
            } catch (_: Throwable) {}

            // 3. Hook registerListener / registerListenerImpl
            val regMethods = listOf("registerListener", "registerListenerImpl")
            for (methodName in regMethods) {
                try {
                    XposedHelpers.hookAllMethods(managerClass, methodName) { chain, _ ->
                        val args = chain.args
                        var listener: Any? = null
                        var sensor: Sensor? = null
                        var handler: Handler? = null

                        for (arg in args) {
                            if (arg != null) {
                                if (arg is SensorEventListener || LocationHooker.hasTypeByName(arg.javaClass, "android.hardware.SensorEventListener")) {
                                    listener = arg
                                } else if (arg is Sensor) {
                                    sensor = arg
                                } else if (arg is Handler) {
                                    handler = arg
                                }
                            }
                        }

                        if (listener != null) {
                            val targetSensor = sensor ?: getOrCreateMockSensor(Sensor.TYPE_STEP_COUNTER, classLoader)
                            if (targetSensor != null && (targetSensor.type == Sensor.TYPE_STEP_COUNTER || targetSensor.type == Sensor.TYPE_STEP_DETECTOR || targetSensor.type == Sensor.TYPE_ACCELEROMETER)) {
                                val entry = CapturedSensorListener(listener, targetSensor, handler)
                                if (!capturedListeners.any { it.listener === listener && it.sensor?.type == targetSensor.type }) {
                                    capturedListeners.add(entry)
                                }

                                val handle = getSensorHandle(targetSensor)
                                if (handle != null) {
                                    handleToTypeMap[handle] = targetSensor.type
                                }

                                // 动态 Hook 该 Listener 的具体实现类（或其基类）中的 onSensorChanged
                                hookConcreteListenerClass(listener.javaClass, classLoader)
                            }
                        }

                        return@hookAllMethods chain.proceed(chain.args.toTypedArray())
                    }
                } catch (_: Throwable) {}
            }

            // 4. Hook unregisterListener / unregisterListenerImpl
            val unregMethods = listOf("unregisterListener", "unregisterListenerImpl")
            for (methodName in unregMethods) {
                try {
                    XposedHelpers.hookAllMethods(managerClass, methodName) { chain, _ ->
                        val args = chain.args
                        val listener = args.firstOrNull {
                            it != null && (it is SensorEventListener || LocationHooker.hasTypeByName(it.javaClass, "android.hardware.SensorEventListener"))
                        }
                        // unregisterListener(listener, sensor) 只取消这一个传感器；不带 sensor 的重载才全部取消。
                        // 之前不看 sensor 参数，App 只是取消加速度计订阅就把它自己的计步订阅一起删了。
                        val sensor = args.firstOrNull { it is Sensor } as? Sensor
                        if (listener != null) {
                            if (sensor != null) {
                                capturedListeners.removeAll {
                                    it.listener === listener && it.sensor?.type == sensor.type
                                }
                            } else {
                                capturedListeners.removeAll { it.listener === listener }
                            }
                        }
                        return@hookAllMethods chain.proceed(chain.args.toTypedArray())
                    }
                } catch (_: Throwable) {}
            }
        }

        // 5. Hook SystemSensorManager$SensorEventQueue.dispatchSensorEvent 底层原生分发接口
        hookSensorEventQueue(classLoader)
    }

    private fun hookSensorEventQueue(classLoader: ClassLoader) {
        try {
            val eventQueueClass = XposedHelpers.findClassIfExists("android.hardware.SystemSensorManager\$SensorEventQueue", classLoader)
                ?: XposedHelpers.findClassIfExists("android.hardware.SystemSensorManager\$BaseEventQueue", classLoader)
                ?: return

            XposedHelpers.hookAllMethods(eventQueueClass, "dispatchSensorEvent") { chain, _ ->
                val config = (XposedHelpers.module as? LocationHooker)?.readConfig()
                if (config != null && config.optBoolean("active", false) && config.optBoolean("enable_step_simulation", true)) {
                    val handle = chain.args.firstOrNull() as? Int
                    val values = chain.args.getOrNull(1) as? FloatArray

                    if (handle != null && values != null) {
                        val sensorType = handleToTypeMap[handle]
                        val now = System.currentTimeMillis()
                        // 速度取"位置推出来的速度"：到达终点 / 暂停时它是 0，配置里的 speed_m_s 不是
                        val speed = effectiveSpeed(config, now)

                        when {
                            sensorType == Sensor.TYPE_STEP_COUNTER && values.isNotEmpty() ->
                                values[0] = calculateCurrentStepsFloat(config, now, speed).toFloat()

                            // 检测器是"每走一步报一次"：只保证值为 1，补发在每秒 tick 里做
                            sensorType == Sensor.TYPE_STEP_DETECTOR && values.isNotEmpty() ->
                                if (speed > MOTION_SPEED) values[0] = 1.0f

                            // 加速度计：仅在真实运动时叠加生理跑步/步行震动特征，静止和到达终点后绝不篡改
                            sensorType == Sensor.TYPE_ACCELEROMETER && values.size >= 3 && speed > MOTION_SPEED ->
                                applySyntheticVibration(values, config, now, speed)
                        }
                    }
                }
                return@hookAllMethods chain.proceed(chain.args.toTypedArray())
            }
        } catch (_: Throwable) {}
    }

    internal fun saveReloadState(): Array<Any?> = arrayOf(
        capturedListeners.map { arrayOf<Any?>(it.listener, it.sensor, it.handler) }.toTypedArray(),
        stepBase, lastStepsFloat, lastStepInitTime,
        mockStepCounterSensor, mockStepDetectorSensor, mockAccelerometerSensor,
        handleToTypeMap.map { arrayOf(it.key, it.value) }.toTypedArray(),
        lastStepComputedAt, lastDetectorStep
    )

    internal fun clearReloadState() { capturedListeners.clear(); hookedListenerClasses.clear() }

    internal fun restoreReloadState(value: Any?) {
        val state = value as? Array<*> ?: return
        (state.getOrNull(0) as? Array<*>)?.filterIsInstance<Array<*>>()?.forEach { entry ->
            val listener = entry.getOrNull(0) ?: return@forEach
            capturedListeners.add(CapturedSensorListener(listener, entry.getOrNull(1) as? Sensor, entry.getOrNull(2) as? Handler))
            hookConcreteListenerClass(
                listener.javaClass,
                listener.javaClass.classLoader ?: ClassLoader.getSystemClassLoader()
            )
        }
        // 老版本保存的状态数组没有后面两项，逐项判空，避免热重载跨版本时 ClassCastException
        (state.getOrNull(1) as? Double)?.let { stepBase = it }
        (state.getOrNull(2) as? Double)?.let { lastStepsFloat = it }
        (state.getOrNull(3) as? Long)?.let { lastStepInitTime = it }
        mockStepCounterSensor = state.getOrNull(4) as? Sensor
        mockStepDetectorSensor = state.getOrNull(5) as? Sensor
        mockAccelerometerSensor = state.getOrNull(6) as? Sensor
        (state.getOrNull(7) as? Array<*>)?.filterIsInstance<Array<*>>()?.forEach {
            val handle = it.getOrNull(0) as? Int ?: return@forEach
            val type = it.getOrNull(1) as? Int ?: return@forEach
            handleToTypeMap[handle] = type
        }
        (state.getOrNull(8) as? Long)?.let { lastStepComputedAt = it }
        (state.getOrNull(9) as? Double)?.let { lastDetectorStep = it }
    }

    /**
     * 动态 Hook 具体的 Listener 实现类（如 Keep 的 StepListener）。
     *
     * 沿继承链找到真正声明了 onSensorChanged 的类再 Hook：不少 App 把实现放在抽象基类里，
     * 只用 `clazz.declaredMethods` 会漏掉，之后只能靠 dispatchSensorEvent 那条路兜底。
     */
    private fun hookConcreteListenerClass(clazz: Class<*>, classLoader: ClassLoader) {
        var candidate: Class<*>? = clazz
        var declaring: Class<*>? = null
        while (candidate != null) {
            if (candidate.declaredMethods.any { it.name == "onSensorChanged" }) {
                declaring = candidate
                break
            }
            candidate = candidate.superclass
        }
        val target = declaring ?: return

        if (hookedListenerClasses.putIfAbsent(target, true) != null) return
        try {
            XposedHelpers.hookAllMethods(target, "onSensorChanged") { chain, _ ->
                val event = chain.args.firstOrNull() as? SensorEvent
                if (event != null && event.sensor != null) {
                    val config = (XposedHelpers.module as? LocationHooker)?.readConfig()
                    if (config != null && config.optBoolean("active", false) && config.optBoolean("enable_step_simulation", true)) {
                        val now = System.currentTimeMillis()
                        val speed = effectiveSpeed(config, now)
                        when {
                            event.sensor.type == Sensor.TYPE_STEP_COUNTER && event.values.isNotEmpty() -> {
                                event.values[0] = calculateCurrentStepsFloat(config, now, speed).toFloat()
                                event.timestamp = SystemClock.elapsedRealtimeNanos()
                            }

                            event.sensor.type == Sensor.TYPE_STEP_DETECTOR && event.values.isNotEmpty() -> {
                                if (speed > MOTION_SPEED) {
                                    event.values[0] = 1.0f
                                    event.timestamp = SystemClock.elapsedRealtimeNanos()
                                }
                            }

                            event.sensor.type == Sensor.TYPE_ACCELEROMETER && event.values.size >= 3 && speed > MOTION_SPEED -> {
                                applySyntheticVibration(event.values, config, now, speed)
                                event.timestamp = SystemClock.elapsedRealtimeNanos()
                            }
                        }
                    }
                }
                return@hookAllMethods chain.proceed(chain.args.toTypedArray())
            }
        } catch (_: Throwable) {}
    }

    /** 用路线 / 摇杆推算出的当前速度；它会在到达终点、暂停时归零，配置里的 speed_m_s 不会 */
    private fun effectiveSpeed(config: JSONObject, now: Long): Double = try {
        RouteEngine.calculateCurrentPosition(config, now).speed.toDouble()
    } catch (_: Throwable) {
        config.optDouble("speed_m_s", 0.0)
    }

    private fun cadenceSpm(config: JSONObject, speed: Double): Double =
        if (config.optBoolean("is_auto_cadence", true)) {
            calculateAutoCadence(speed).toDouble()
        } else {
            config.optInt("step_cadence_spm", 165).coerceIn(60, 240).toDouble()
        }

    private fun applySyntheticVibration(values: FloatArray, config: JSONObject, now: Long, speed: Double) {
        if (values.size < 3) return
        val stepsFloat = calculateCurrentStepsFloat(config, now, speed)
        val synthetic = RouteEngine.realismSession(config).accelerometer(
            stepsFloat,
            speed,
            gaitTemplate(config),
            noiseRng
        )
        if (synthetic.size < 3) return
        values[0] = synthetic[0]
        values[1] = synthetic[1]
        values[2] = synthetic[2]
    }

    /** 用户录制的步态模板（配置里的编码字符串），按字符串内容缓存解码结果 */
    private fun gaitTemplate(config: JSONObject): GaitTemplate? {
        val source = config.optString("gait_template", "")
        if (source != cachedTemplateSource) {
            cachedTemplate = GaitTemplate.decode(source)
            cachedTemplateSource = source
        }
        return cachedTemplate
    }

    /**
     * 计算当前仿真总步数
     */
    fun calculateCurrentSteps(config: JSONObject, now: Long = System.currentTimeMillis()): Long =
        calculateCurrentStepsFloat(config, now).toLong()

    /**
     * 连续的累计步数：整数部分是步数，小数部分是当前这一步的相位，供加速度波形对齐。
     *
     * 所有共享状态都在 [stepLock] 内读写；不要拆成无锁的读-改-写，
     * 传感器线程和主线程并发时会把累计步数改回去。
     */
    private fun calculateCurrentStepsFloat(
        config: JSONObject,
        now: Long,
        speedOverride: Double? = null
    ): Double {
        val speed = speedOverride ?: effectiveSpeed(config, now)
        val cadence = cadenceSpm(config, speed)

        return synchronized(stepLock) {
            // 老配置 / 手写配置可能没有 start_timestamp：退化成"第一次见到的锚点"，
            // 不能每次取 now，否则每调用一次都重新起算，步数会完全冻结
            var startTime = config.optLong("start_timestamp", 0L)
            if (startTime <= 0L) startTime = if (lastStepInitTime > 0L) lastStepInitTime else now

            if (lastStepInitTime != startTime) {
                // 摇杆模式每秒重写 start_timestamp：把 [上次计算时刻, 新起点] 这段按步频补进基准，
                // 否则每重写一次就丢一段；暂停 / 进程空闲造成的长间隔不补
                if (lastStepInitTime > 0L && speed > STATIONARY_SPEED) {
                    val gapMs = (startTime - lastStepComputedAt).coerceIn(0L, MAX_STEP_GAP_MS)
                    if (gapMs > 0L) {
                        stepBase = lastStepsFloat + cadence / 60.0 * (gapMs / 1000.0)
                    }
                }
                lastStepInitTime = startTime
            }

            if (speed <= STATIONARY_SPEED) {
                lastStepComputedAt = now
                lastStepsFloat
            } else {
                val elapsedSec = ((now - startTime).coerceAtLeast(0L)) / 1000.0
                val steps = stepBase + RouteEngine.realismSession(config, startTime).steps(cadence, elapsedSec)
                lastStepsFloat = steps
                lastStepComputedAt = now
                steps
            }
        }
    }

    /**
     * 智能根据速度自动计算匹配的生理步频 (SPM, 步/分钟)
     */
    fun calculateAutoCadence(speedMs: Double): Int {
        return when {
            speedMs <= 0.8 -> 95 // 慢走
            speedMs <= 1.5 -> (100 + (speedMs - 0.8) / 0.7 * 25).toInt() // 快走 100~125 SPM
            speedMs <= 2.8 -> (135 + (speedMs - 1.5) / 1.3 * 25).toInt() // 慢跑 135~160 SPM
            speedMs <= 4.0 -> (160 + (speedMs - 2.8) / 1.2 * 20).toInt() // 匀速跑 160~180 SPM
            speedMs <= 5.5 -> (180 + (speedMs - 4.0) / 1.5 * 15).toInt() // 快跑 180~195 SPM
            else -> 200 // 冲刺 200 SPM
        }
    }

    /**
     * 由 ConfigPoller 每秒主动向监听器推送计步事件（仅在运动路线模拟中生效）
     */
    fun dispatchStepEvents(config: JSONObject, classLoader: ClassLoader) {
        if (!config.optBoolean("active", false)) return
        val enableStep = config.optBoolean("enable_step_simulation", true)
        if (!enableStep) return

        val now = System.currentTimeMillis()
        val speed = effectiveSpeed(config, now)
        // 静态定位时绝不主动推送步频和加速度事件，保证主线程和人脸识别传感器纯净
        if (speed <= STATIONARY_SPEED) return

        val totalSteps = calculateCurrentStepsFloat(config, now, speed)
        // 检测器一个 tick 内可能要走 2~3 步；按实际新增步数补发，而不是固定 1 条
        val detectorDelta = nextDetectorDelta(totalSteps)

        val listeners = capturedListeners.toList()
        if (listeners.isEmpty()) return

        for (entry in listeners) {
            val listener = entry.listener
            val sensor = entry.sensor ?: continue
            val targetHandler = entry.handler ?: mainLooperHandler ?: continue

            when (sensor.type) {
                Sensor.TYPE_STEP_COUNTER -> {
                    postSensorEvent(targetHandler, listener, sensor, floatArrayOf(totalSteps.toFloat()))
                }

                Sensor.TYPE_STEP_DETECTOR -> {
                    if (speed > MOTION_SPEED) {
                        repeat(detectorDelta) {
                            postSensorEvent(targetHandler, listener, sensor, floatArrayOf(1.0f))
                        }
                    }
                }

                Sensor.TYPE_ACCELEROMETER -> {
                    if (speed > MOTION_SPEED) {
                        val rawValues = floatArrayOf(0f, 0f, 9.8f)
                        applySyntheticVibration(rawValues, config, now, speed)
                        postSensorEvent(targetHandler, listener, sensor, rawValues)
                    }
                }
            }
        }
    }

    /**
     * 本 tick 应该补发几步的检测器事件。
     * 第一次调用（或进程刚重启）只对齐基线，不补发，避免一启动就吐出 6 步。
     */
    private fun nextDetectorDelta(totalSteps: Double): Int = synchronized(stepLock) {
        val whole = floor(totalSteps)
        if (lastDetectorStep <= 0.0) {
            lastDetectorStep = whole
            0
        } else {
            val delta = (whole - lastDetectorStep).toInt()
            if (delta <= 0) {
                0
            } else {
                lastDetectorStep = whole
                delta.coerceAtMost(MAX_DETECTOR_BURST)
            }
        }
    }

    private fun postSensorEvent(handler: Handler, listener: Any, sensor: Sensor, values: FloatArray) {
        val event = createSensorEvent(sensor, values) ?: return
        handler.post {
            try {
                if (listener is SensorEventListener) {
                    listener.onSensorChanged(event)
                } else {
                    XposedHelpers.callMethod(listener, "onSensorChanged", event)
                }
            } catch (_: Throwable) {}
        }
    }

    private fun getSensorHandle(sensor: Sensor): Int? {
        return try {
            val handleField = Sensor::class.java.getDeclaredField("mHandle")
            handleField.isAccessible = true
            handleField.getInt(sensor)
        } catch (_: Throwable) {
            null
        }
    }

    private fun getOrCreateMockSensor(type: Int, classLoader: ClassLoader): Sensor? {
        if (type == Sensor.TYPE_STEP_COUNTER && mockStepCounterSensor != null) return mockStepCounterSensor
        if (type == Sensor.TYPE_STEP_DETECTOR && mockStepDetectorSensor != null) return mockStepDetectorSensor
        if (type == Sensor.TYPE_ACCELEROMETER && mockAccelerometerSensor != null) return mockAccelerometerSensor

        try {
            val sensorClass = Class.forName("android.hardware.Sensor", false, classLoader)
            val constructor: Constructor<*> = sensorClass.getDeclaredConstructor()
            constructor.isAccessible = true
            val sensor = constructor.newInstance() as Sensor

            setSensorField(sensor, "mType", type)
            setSensorField(sensor, "mName", when(type) {
                Sensor.TYPE_STEP_COUNTER -> "Step Counter Sensor"
                Sensor.TYPE_STEP_DETECTOR -> "Step Detector Sensor"
                else -> "Accelerometer Sensor"
            })
            setSensorField(sensor, "mVendor", "Android")
            setSensorField(sensor, "mVersion", 1)
            setSensorField(sensor, "mResolution", 1.0f)
            setSensorField(sensor, "mPower", 0.05f)
            // 只填前面几个字段的话 getMaxRange()/getMinDelay()/getStringType() 都是默认值，
            // 按 stringType 或 maxRange 判断能力的 App 会直接放弃这个传感器
            setSensorField(sensor, "mMaxRange", 1.0f)
            setSensorField(sensor, "mMinDelay", 0)
            setSensorField(sensor, "mStringType", when (type) {
                Sensor.TYPE_STEP_COUNTER -> Sensor.STRING_TYPE_STEP_COUNTER
                Sensor.TYPE_STEP_DETECTOR -> Sensor.STRING_TYPE_STEP_DETECTOR
                else -> Sensor.STRING_TYPE_ACCELEROMETER
            })
            // 计步器是 on-change(2)，检测器是事件型(3)；只影响上报模式查询，不影响事件投递
            setSensorField(sensor, "mReportingMode", if (type == Sensor.TYPE_STEP_DETECTOR) 3 else 2)

            when (type) {
                Sensor.TYPE_STEP_COUNTER -> mockStepCounterSensor = sensor
                Sensor.TYPE_STEP_DETECTOR -> mockStepDetectorSensor = sensor
                Sensor.TYPE_ACCELEROMETER -> mockAccelerometerSensor = sensor
            }
            return sensor
        } catch (_: Throwable) {
            return null
        }
    }

    private fun setSensorField(sensor: Sensor, fieldName: String, value: Any) {
        try {
            val field: Field = Sensor::class.java.getDeclaredField(fieldName)
            field.isAccessible = true
            field.set(sensor, value)
        } catch (_: Throwable) {}
    }

    private fun createSensorEvent(sensor: Sensor, values: FloatArray): SensorEvent? {
        try {
            val eventConstructor = SensorEvent::class.java.getDeclaredConstructor(Int::class.javaPrimitiveType)
            eventConstructor.isAccessible = true
            val event = eventConstructor.newInstance(values.size) as SensorEvent
            System.arraycopy(values, 0, event.values, 0, values.size)

            val sensorField = SensorEvent::class.java.getDeclaredField("sensor")
            sensorField.isAccessible = true
            sensorField.set(event, sensor)

            val timestampField = SensorEvent::class.java.getDeclaredField("timestamp")
            timestampField.isAccessible = true
            timestampField.setLong(event, SystemClock.elapsedRealtimeNanos())

            val accuracyField = SensorEvent::class.java.getDeclaredField("accuracy")
            accuracyField.isAccessible = true
            accuracyField.setInt(event, 3) // SENSOR_STATUS_ACCURACY_HIGH

            return event
        } catch (_: Throwable) {
            try {
                // Fallback 1: search any constructors
                val constructors = SensorEvent::class.java.declaredConstructors
                for (ctor in constructors) {
                    ctor.isAccessible = true
                    val paramTypes = ctor.parameterTypes
                    val args = arrayOfNulls<Any>(paramTypes.size)
                    for (i in args.indices) {
                        if (paramTypes[i] == Int::class.javaPrimitiveType) args[i] = values.size
                    }
                    val event = ctor.newInstance(*args) as SensorEvent
                    System.arraycopy(values, 0, event.values, 0, values.size)
                    try {
                        val sf = SensorEvent::class.java.getDeclaredField("sensor")
                        sf.isAccessible = true
                        sf.set(event, sensor)
                        val tf = SensorEvent::class.java.getDeclaredField("timestamp")
                        tf.isAccessible = true
                        tf.setLong(event, SystemClock.elapsedRealtimeNanos())
                        val af = SensorEvent::class.java.getDeclaredField("accuracy")
                        af.isAccessible = true
                        af.setInt(event, 3)
                    } catch (_: Throwable) {}
                    return event
                }
            } catch (_: Throwable) {}

            try {
                // Fallback 2: sun.misc.Unsafe.allocateInstance
                val unsafeClass = Class.forName("sun.misc.Unsafe")
                val theUnsafeField = unsafeClass.getDeclaredField("theUnsafe")
                theUnsafeField.isAccessible = true
                val unsafe = theUnsafeField.get(null)
                val allocateMethod = unsafeClass.getMethod("allocateInstance", Class::class.java)
                val event = allocateMethod.invoke(unsafe, SensorEvent::class.java) as SensorEvent

                val valuesField = SensorEvent::class.java.getDeclaredField("values")
                valuesField.isAccessible = true
                val arr = FloatArray(values.size)
                System.arraycopy(values, 0, arr, 0, values.size)
                valuesField.set(event, arr)

                val sf = SensorEvent::class.java.getDeclaredField("sensor")
                sf.isAccessible = true
                sf.set(event, sensor)

                val tf = SensorEvent::class.java.getDeclaredField("timestamp")
                tf.isAccessible = true
                tf.setLong(event, SystemClock.elapsedRealtimeNanos())

                val af = SensorEvent::class.java.getDeclaredField("accuracy")
                af.isAccessible = true
                af.setInt(event, 3)
                return event
            } catch (_: Throwable) {}

            return null
        }
    }
}
