package com.example.navgraph.ui

import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import android.graphics.Path
import android.graphics.Typeface
import android.view.View
import com.example.navgraph.R
import com.example.navgraph.estimator.HorizontalPoint
import com.example.navgraph.estimator.ReplayResult
import kotlin.math.max
import kotlin.math.min

class ReplayTrajectoryView(context: Context) : View(context) {
    private val gridPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = context.getColor(R.color.line)
        strokeWidth = resources.displayMetrics.density
    }
    private val truthPaint = routePaint(context.getColor(R.color.ink), 2f)
    private val wlsPaint = routePaint(context.getColor(R.color.red), 2f)
    private val fgoPaint = routePaint(context.getColor(R.color.teal), 3f)
    private val labelPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = context.getColor(R.color.muted)
        textSize = 11f * resources.displayMetrics.scaledDensity
        typeface = Typeface.DEFAULT
    }
    private var truth = emptyList<HorizontalPoint>()
    private var wls = emptyList<HorizontalPoint>()
    private var fgo = emptyList<HorizontalPoint>()

    fun setTrajectories(result: ReplayResult) {
        truth = result.truthTrajectory
        wls = result.wlsTrajectory
        fgo = result.fgoTrajectory
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
            val x = left + (right - left) * fraction
            val y = top + (bottom - top) * fraction
            canvas.drawLine(x, top, x, bottom, gridPaint)
            canvas.drawLine(left, y, right, y, gridPaint)
        }
        val all = truth + wls + fgo
        if (all.isEmpty()) return
        var minEast = all.minOf { it.eastMeters }
        var maxEast = all.maxOf { it.eastMeters }
        var minNorth = all.minOf { it.northMeters }
        var maxNorth = all.maxOf { it.northMeters }
        val span = max(maxEast - minEast, maxNorth - minNorth).coerceAtLeast(1.0)
        val centerEast = (minEast + maxEast) / 2.0
        val centerNorth = (minNorth + maxNorth) / 2.0
        minEast = centerEast - span / 2.0
        maxEast = centerEast + span / 2.0
        minNorth = centerNorth - span / 2.0
        maxNorth = centerNorth + span / 2.0

        fun path(points: List<HorizontalPoint>): Path = Path().apply {
            points.forEachIndexed { index, point ->
                val x = left + ((point.eastMeters - minEast) / (maxEast - minEast) * (right - left)).toFloat()
                val y = bottom - ((point.northMeters - minNorth) / (maxNorth - minNorth) * (bottom - top)).toFloat()
                if (index == 0) moveTo(x, y) else lineTo(x, y)
            }
        }
        canvas.drawPath(path(truth), truthPaint)
        canvas.drawPath(path(wls), wlsPaint)
        canvas.drawPath(path(fgo), fgoPaint)
        legend(canvas, 8f * density, 14f * density, "Truth", truthPaint)
        legend(canvas, 82f * density, 14f * density, "WLS", wlsPaint)
        legend(canvas, 139f * density, 14f * density, "FGO", fgoPaint)
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
        canvas.drawLine(x, y, x + 17f * density, y, paint)
        canvas.drawText(label, x + 22f * density, y + 4f * density, labelPaint)
    }
}
