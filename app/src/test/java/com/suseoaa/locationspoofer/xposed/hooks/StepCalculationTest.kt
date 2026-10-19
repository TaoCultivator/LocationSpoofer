package com.suseoaa.locationspoofer.xposed.hooks

import org.junit.Assert.assertEquals
import org.junit.Test

class StepCalculationTest {

    private val now = 1_760_000_000_000L

    @Test
    fun `zero start timestamp never counts from epoch`() {
        assertEquals(2350L, calculateSteps(2350L, 0L, now, 165.0))
    }

    @Test
    fun `one minute at 165 spm adds exactly 165 steps`() {
        assertEquals(2515L, calculateSteps(2350L, now - 60_000L, now, 165.0))
    }

    @Test
    fun `elapsed time is converted from milliseconds to seconds`() {
        assertEquals(2370L, calculateSteps(2350L, now - 10_000L, now, 120.0))
    }

    @Test
    fun `future start timestamp does not produce negative steps`() {
        assertEquals(2350L, calculateSteps(2350L, now + 60_000L, now, 165.0))
    }

    @Test
    fun `negative start timestamp is treated as invalid`() {
        assertEquals(2350L, calculateSteps(2350L, -1L, now, 165.0))
    }
}
