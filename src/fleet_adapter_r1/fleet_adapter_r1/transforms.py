# Copyright 2026 fleet_adapter_r1 contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""2D similarity-transform helper (rotation + uniform scale + translation).

This replaces the third-party ``nudged`` dependency used by the upstream RMF
fleet adapter templates. ``estimate_transform`` solves the least-squares
similarity transform mapping ``domain`` points onto ``range`` points using a
closed-form complex-number formulation, which returns the identity transform
when the two point sets are identical (the default for this adapter).
"""

import math


class Transform:
    """A 2D similarity transform: p -> scale * R(rotation) * p + translation."""

    def __init__(self, rotation=0.0, scale=1.0, translation=(0.0, 0.0)):
        self.rotation = float(rotation)
        self.scale = float(scale)
        self.translation = (float(translation[0]), float(translation[1]))

    def transform(self, xy):
        c = self.scale * math.cos(self.rotation)
        s = self.scale * math.sin(self.rotation)
        x, y = float(xy[0]), float(xy[1])
        return [c * x - s * y + self.translation[0],
                s * x + c * y + self.translation[1]]

    def get_rotation(self):
        return self.rotation

    def get_scale(self):
        return self.scale

    def get_translation(self):
        return list(self.translation)


def estimate_transform(domain, rng):
    """Estimate a similarity transform mapping ``domain`` onto ``rng``.

    ``domain`` and ``rng`` are equal-length lists of ``[x, y]`` pairs. Returns a
    :class:`Transform`. With identical inputs this yields the identity transform.
    """
    n = min(len(domain), len(rng))
    if n == 0:
        return Transform()
    a = [complex(float(p[0]), float(p[1])) for p in domain[:n]]
    b = [complex(float(p[0]), float(p[1])) for p in rng[:n]]
    abar = sum(a) / n
    bbar = sum(b) / n
    # Complex least squares for b = c * a + t  ->  c = Cov(a, b) / Var(a)
    num = sum((ai - abar).conjugate() * (bi - bbar) for ai, bi in zip(a, b))
    den = sum(abs(ai - abar) ** 2 for ai in a)
    c = complex(1.0, 0.0) if den < 1e-12 else num / den
    t = bbar - c * abar
    return Transform(rotation=math.atan2(c.imag, c.real),
                     scale=abs(c),
                     translation=(t.real, t.imag))
