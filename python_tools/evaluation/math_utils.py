"""Small dependency-free numerical helpers for replay evaluation."""

from __future__ import annotations

import math
from typing import Sequence

Vec3 = tuple[float, float, float]


def add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a: Vec3, value: float) -> Vec3:
    return (a[0] * value, a[1] * value, a[2] * value)


def dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def norm(a: Vec3) -> float:
    return math.sqrt(dot(a, a))


def distance(a: Vec3, b: Vec3) -> float:
    return norm(sub(a, b))


def normalize(a: Vec3) -> Vec3:
    n = norm(a)
    if n <= 0.0:
        raise ValueError("cannot normalize zero vector")
    return scale(a, 1.0 / n)


def rms(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(value * value for value in values) / len(values))


def solve_linear_system(matrix: list[list[float]], rhs: list[float]) -> list[float]:
    n = len(matrix)
    if any(len(row) != n for row in matrix) or len(rhs) != n:
        raise ValueError("expected square matrix and matching rhs")

    a = [row[:] for row in matrix]
    b = rhs[:]
    for pivot in range(n):
        best = max(range(pivot, n), key=lambda row: abs(a[row][pivot]))
        if abs(a[best][pivot]) < 1e-18:
            raise ValueError("singular linear system")
        if best != pivot:
            a[pivot], a[best] = a[best], a[pivot]
            b[pivot], b[best] = b[best], b[pivot]
        diagonal = a[pivot][pivot]
        for col in range(pivot, n):
            a[pivot][col] /= diagonal
        b[pivot] /= diagonal
        for row in range(n):
            if row == pivot:
                continue
            factor = a[row][pivot]
            if factor == 0.0:
                continue
            for col in range(pivot, n):
                a[row][col] -= factor * a[pivot][col]
            b[row] -= factor * b[pivot]
    return b


def solve_normal_equations(jacobian: list[list[float]], residuals: list[float], damping: float) -> list[float]:
    if not jacobian:
        raise ValueError("empty Jacobian")
    rows = len(jacobian)
    cols = len(jacobian[0])
    normal = [[0.0 for _ in range(cols)] for _ in range(cols)]
    rhs = [0.0 for _ in range(cols)]
    for row in range(rows):
        for i in range(cols):
            rhs[i] -= jacobian[row][i] * residuals[row]
            for j in range(cols):
                normal[i][j] += jacobian[row][i] * jacobian[row][j]
    for i in range(cols):
        normal[i][i] += damping
    return solve_linear_system(normal, rhs)
