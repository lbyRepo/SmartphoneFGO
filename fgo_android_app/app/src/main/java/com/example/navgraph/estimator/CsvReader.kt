package com.example.navgraph.estimator

import java.io.InputStream

internal object CsvReader {
    fun read(input: InputStream): List<Map<String, String>> = input.bufferedReader().use { reader ->
        val iterator = reader.lineSequence().iterator()
        if (!iterator.hasNext()) return@use emptyList()
        val header = parseLine(iterator.next())
        buildList {
            while (iterator.hasNext()) {
                val values = parseLine(iterator.next())
                add(header.indices.associate { index ->
                    header[index] to values.getOrElse(index) { "" }
                })
            }
        }
    }

    private fun parseLine(line: String): List<String> {
        val values = mutableListOf<String>()
        val current = StringBuilder()
        var quoted = false
        var index = 0
        while (index < line.length) {
            val character = line[index]
            when {
                character == '"' && quoted && index + 1 < line.length && line[index + 1] == '"' -> {
                    current.append('"')
                    index++
                }
                character == '"' -> quoted = !quoted
                character == ',' && !quoted -> {
                    values += current.toString()
                    current.clear()
                }
                else -> current.append(character)
            }
            index++
        }
        values += current.toString()
        return values
    }
}
