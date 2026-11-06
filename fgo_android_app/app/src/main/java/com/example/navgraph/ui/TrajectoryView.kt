package com.example.navgraph.ui

import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import android.graphics.Path
import android.graphics.Typeface
import android.view.View
import com.example.navgraph.R
import kotlin.math.cos

class TrajectoryView(context: Context) : View(context) {
    private val gridPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = context.getColor(R.color.line)
        strokeWidth = resources.displayMetrics.density
    }
    private val routePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = context.getColor(R.color.teal)
        strokeWidth = 3f * resources.displayMetrics.density
        style = Paint.Style.STROKE
        strokeCap = Paint.Cap.ROUND
        strokeJoin = Paint.Join.ROUND
    }
    private val pointPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = context.getColor(R.color.teal)
        style = Paint.Style.FILL
    }
    private val labelPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = context.getColor(R.color.muted)
        textSize = 12f * resources.displayMetrics.scaledDensity
        typeface = Typeface.create(Typeface.DEFAULT, Typeface.NORMAL)
    }
    private val routePath = Path()
    private val points = ArrayDeque<Pair<Double, Double>>()

    fun addLocation(latitude: Double, longitude: Double) {
        points.addLast(latitude to longitude)
        while (points.size > 240) points.removeFirst()
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val left = paddingLeft.toFloat()
        val top = paddingTop.toFloat()
        val right = (width - paddingRight).toFloat()
        val bottom = (height - paddingBottom).toFloat()
        val step = 40f * resources.displayMetrics.density
        var x = left
        while (x <= right) {
            canvas.drawLine(x, top, x, bottom, gridPaint)
            x += step
        }
        var y = top
        while (y <= bottom) {
            canvas.drawLine(left, y, right, y, gridPaint)
            y += step
        }

        if (points.isEmpty()) {
            val centerX = (left + right) / 2f
            val centerY = (top + bottom) / 2f
            canvas.drawCircle(centerX, centerY, 7f * resources.displayMetrics.density, pointPaint)
            canvas.drawText("Awaiting position", left + 16f * resources.displayMetrics.density,
                bottom - 16f * resources.displayMetrics.density, labelPaint)
            return
        }

        val origin = points.first()
        val metersPerDegreeLat = 111_132.0
        val metersPerDegreeLon = 111_320.0 * cos(Math.toRadians(origin.first))
        val local = points.map { point ->
            ((point.second - origin.second) * metersPerDegreeLon) to
                ((point.first - origin.first) * metersPerDegreeLat)
        }
        val minX = local.minOf { it.first }
        val maxX = local.maxOf { it.first }
        val minY = local.minOf { it.second }
        val maxY = local.maxOf { it.second }
        val span = maxOf(maxX - minX, maxY - minY, 10.0)
        val usableWidth = right - left - 48f * resources.displayMetrics.density
        val usableHeight = bottom - top - 48f * resources.displayMetrics.density
        val scale = minOf(usableWidth, usableHeight) / span.toFloat()
        routePath.reset()
        local.forEachIndexed { index, point ->
            val px = (left + right) / 2f + ((point.first - (minX + maxX) / 2.0) * scale).toFloat()
            val py = (top + bottom) / 2f - ((point.second - (minY + maxY) / 2.0) * scale).toFloat()
            if (index == 0) routePath.moveTo(px, py) else routePath.lineTo(px, py)
            if (index == local.lastIndex) canvas.drawCircle(px, py, 6f * resources.displayMetrics.density, pointPaint)
        }
        canvas.drawPath(routePath, routePaint)
        canvas.drawText("${points.size} fixes", left + 16f * resources.displayMetrics.density,
            bottom - 16f * resources.displayMetrics.density, labelPaint)
    }
}
