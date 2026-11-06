package com.example.s23logger

import android.Manifest
import android.app.Activity
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.provider.Settings
import android.view.ViewGroup
import android.widget.Button
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.TextView

class MainActivity : Activity() {
    private lateinit var statusText: TextView
    private lateinit var startButton: Button
    private lateinit var stopButton: Button

    private val statusReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            if (intent.action != LoggingService.ACTION_STATUS) return
            val message = intent.getStringExtra(LoggingService.EXTRA_STATUS_MESSAGE).orEmpty()
            val logPath = intent.getStringExtra(LoggingService.EXTRA_LOG_PATH).orEmpty()
            val suffix = if (logPath.isNotBlank()) "\n\nLog:\n$logPath" else ""
            statusText.text = "$message$suffix"
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        buildUi()
        refreshPermissionStatus()
    }

    override fun onResume() {
        super.onResume()
        val filter = IntentFilter(LoggingService.ACTION_STATUS)
        if (Build.VERSION.SDK_INT >= 33) {
            registerReceiver(statusReceiver, filter, RECEIVER_NOT_EXPORTED)
        } else {
            @Suppress("DEPRECATION")
            registerReceiver(statusReceiver, filter)
        }
    }

    override fun onPause() {
        super.onPause()
        runCatching { unregisterReceiver(statusReceiver) }
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray,
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == REQUEST_PERMISSIONS) {
            refreshPermissionStatus()
        }
    }

    private fun buildUi() {
        val density = resources.displayMetrics.density
        fun dp(value: Int): Int = (value * density).toInt()

        statusText = TextView(this).apply {
            textSize = 15f
            setPadding(dp(16), dp(16), dp(16), dp(16))
        }
        startButton = Button(this).apply {
            text = "Start logging"
            setOnClickListener { startLogging() }
        }
        stopButton = Button(this).apply {
            text = "Stop logging"
            setOnClickListener { stopLogging() }
        }
        val permissionButton = Button(this).apply {
            text = "Grant permissions"
            setOnClickListener { requestMissingPermissions() }
        }
        val settingsButton = Button(this).apply {
            text = "Open app settings"
            setOnClickListener {
                startActivity(Intent(Settings.ACTION_APPLICATION_DETAILS_SETTINGS).apply {
                    data = android.net.Uri.parse("package:$packageName")
                })
            }
        }

        val content = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(16), dp(16), dp(16), dp(16))
            addView(startButton, LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT)
            addView(stopButton, LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT)
            addView(permissionButton, LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT)
            addView(settingsButton, LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT)
            addView(statusText, LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT)
        }
        setContentView(ScrollView(this).apply {
            addView(content, ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT)
        })
    }

    private fun startLogging() {
        if (!hasRequiredPermissions()) {
            requestMissingPermissions()
            return
        }
        val intent = Intent(this, LoggingService::class.java).setAction(LoggingService.ACTION_START)
        if (Build.VERSION.SDK_INT >= 26) {
            startForegroundService(intent)
        } else {
            startService(intent)
        }
    }

    private fun stopLogging() {
        startService(Intent(this, LoggingService::class.java).setAction(LoggingService.ACTION_STOP))
    }

    private fun refreshPermissionStatus() {
        val missing = missingRuntimePermissions()
        startButton.isEnabled = missing.isEmpty()
        statusText.text = if (missing.isEmpty()) {
            "Ready. Logs are written to the app external files directory under Documents/logs."
        } else {
            "Missing runtime permissions:\n${missing.joinToString(separator = "\n")}"
        }
    }

    private fun requestMissingPermissions() {
        val missing = missingRuntimePermissions()
        if (missing.isNotEmpty()) {
            requestPermissions(missing.toTypedArray(), REQUEST_PERMISSIONS)
        }
    }

    private fun hasRequiredPermissions(): Boolean = missingRuntimePermissions().isEmpty()

    private fun missingRuntimePermissions(): List<String> {
        val requested = buildList {
            add(Manifest.permission.ACCESS_FINE_LOCATION)
            if (Build.VERSION.SDK_INT >= 33) {
                add(Manifest.permission.POST_NOTIFICATIONS)
            }
        }
        return requested.filter { checkSelfPermission(it) != PackageManager.PERMISSION_GRANTED }
    }

    companion object {
        private const val REQUEST_PERMISSIONS = 1001
    }
}
