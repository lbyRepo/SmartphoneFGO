package com.example.navgraph.ui

import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import android.graphics.Path
import android.graphics.Typeface
import android.view.View
import com.example.navgraph.R
import com.example.navgraph.estimator.GnssInsValidation
import com.example.navgraph.estimator.HorizontalPoint
import kotlin.math.max

class GnssInsTrajectoryView(context: Context) : View(context) {
    private val gridPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = context.getColor(R.color.line)
        strokeWidth = resources.displayMetrics.density
    }
    private val truthPaint = routePaint(context.getColor(R.color.ink), 2f)
    private val wlsPaint = routePaint(context.getColor(R.color.red), 2f)
    private val ekfPaint = routePaint(context.getColor(R.color.blue), 3f)
    private val fgoPaint = routePaint(context.getColor(R.color.teal), 3f)
    private val labelPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = context.getColor(R.color.muted)
        textSize = 11f * resources.displayMetrics.scaledDensity
        typeface = Typeface.DEFAULT
    }
    private var truth = emptyList<HorizontalPoint>()
    private var wls = emptyList<HorizontalPoint>()
    private var ekf = emptyList<HorizontalPoint>()
    private var fgo = emptyList<HorizontalPoint>()

    fun setTrajectories(validation: GnssInsValidation) {
        truth = validation.truthTrajectory
        wls = validation.wlsTrajectory
        ekf = validation.ekfTrajectory
        fgo = validation.fgoTrajectory
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val density = resources.displayMetrics.density
        val left = 4f * density
        val top = 30f * density
        val right = width - 4f * density
        val bottom = height - 5f * density
        repeat(5) { index ->
            val fraction = index / 4f
            canvas.drawLine(left + (right - left) * fraction, top,
                left + (right - left) * fraction, bottom, gridPaint)
            canvas.drawLine(left, top + (bottom - top) * fraction,
                right, top + (bottom - top) * fraction, gridPaint)
        }
        val all = truth + wls + ekf + fgo
        if (all.isEmpty()) return
        val rawMinEast = all.minOf { it.eastMeters }
        val rawMaxEast = all.maxOf { it.eastMeters }
        val rawMinNorth = all.minOf { it.northMeters }
        val rawMaxNorth = all.maxOf { it.northMeters }
        val span = max(rawMaxEast - rawMinEast, rawMaxNorth - rawMinNorth).coerceAtLeast(1.0)
        val centerEast = (rawMinEast + rawMaxEast) / 2.0
        val centerNorth = (rawMinNorth + rawMaxNorth) / 2.0
        val minEast = centerEast - span / 2.0
        val minNorth = centerNorth - span / 2.0

        fun path(points: List<HorizontalPoint>): Path = Path().apply {
            points.forEachIndexed { index, point ->
                val x = left + ((point.eastMeters - minEast) / span * (right - left)).toFloat()
                val y = bottom - ((point.northMeters - minNorth) / span * (bottom - top)).toFloat()
                if (index == 0) moveTo(x, y) else lineTo(x, y)
            }
        }
        canvas.drawPath(path(truth), truthPaint)
        canvas.drawPath(path(wls), wlsPaint)
        canvas.drawPath(path(ekf), ekfPaint)
        canvas.drawPath(path(fgo), fgoPaint)
        legend(canvas, 8f * density, 14f * density, "Truth", truthPaint)
        legend(canvas, 73f * density, 14f * density, "WLS", wlsPaint)
        legend(canvas, 126f * density, 14f * density, "EKF", ekfPaint)
        legend(canvas, 181f * density, 14f * density, "FGO", fgoPaint)
    }

    private fun routePaint(color: Int, widthDp: Float) = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        this.color = color
        strokeWidth = widthDp * resources.displayMetrics.density
        style = Paint.Style.STROKE
        strokeCap = Paint.Cap.ROUND
        strokeJoin = Paint.Join.ROUND
    }

    private fun legend(canvas: Canvas, x: Float, y: Float, label: String, paint: Paint) {
        val density = resources.displayMetrics.density
        canvas.drawLine(x, y, x + 14f * density, y, paint)
        canvas.drawText(label, x + 18f * density, y + 4f * density, labelPaint)
    }
}
