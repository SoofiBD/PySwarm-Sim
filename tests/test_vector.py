import pytest
import sys
sys.path.insert(0, '.')

from app.myMath.vector import Vector, VectorOperations


class TestVector:
    def test_vector_creation(self):
        v = Vector(1.0, 2.0, 3.0)
        assert v.x == 1.0
        assert v.y == 2.0
        assert v.z == 3.0

    def test_vector_default(self):
        v = Vector()
        assert v.x == 0.0
        assert v.y == 0.0
        assert v.z == 0.0

    def test_get_length(self):
        v = Vector(3.0, 4.0, 0.0)
        assert v.get_length() == 5.0

    def test_normalize(self):
        v = Vector(3.0, 4.0, 0.0)
        normalized = v.normalize()
        assert normalized.get_length() == 1.0

    def test_normalize_zero(self):
        v = Vector(0.0, 0.0, 0.0)
        normalized = v.normalize()
        assert normalized.get_length() == 0.0


class TestVectorOperations:
    def test_sum(self):
        v1 = Vector(1.0, 2.0, 3.0)
        v2 = Vector(4.0, 5.0, 6.0)
        result = VectorOperations.sum(v1, v2)
        assert result.x == 5.0
        assert result.y == 7.0
        assert result.z == 9.0

    def test_substract(self):
        v1 = Vector(4.0, 5.0, 6.0)
        v2 = Vector(1.0, 2.0, 3.0)
        result = VectorOperations.substract(v1, v2)
        assert result.x == 3.0
        assert result.y == 3.0
        assert result.z == 3.0

    def test_multiply(self):
        v = Vector(1.0, 2.0, 3.0)
        result = VectorOperations.multiply(v, 2.0)
        assert result.x == 2.0
        assert result.y == 4.0
        assert result.z == 6.0

    def test_isEqual(self):
        v1 = Vector(1.0, 2.0, 3.0)
        v2 = Vector(1.0, 2.0, 3.0)
        assert VectorOperations.isEqual(v1, v2) is True

    def test_isAlmostEqual(self):
        v1 = Vector(1.0, 2.0, 3.0)
        v2 = Vector(1.1, 2.1, 3.1)
        assert bool(VectorOperations.isAlmostEqual(v1, v2, 0.2)) is True

    def test_isAlmostEqual_false(self):
        v1 = Vector(1.0, 2.0, 3.0)
        v2 = Vector(5.0, 6.0, 7.0)
        assert bool(VectorOperations.isAlmostEqual(v1, v2, 0.5)) is False

    def test_divide(self):
        v = Vector(4.0, 6.0, 8.0)
        result = VectorOperations.divide(v, 2.0)
        assert result.x == 2.0
        assert result.y == 3.0
        assert result.z == 4.0

    def test_divide_by_zero(self):
        v = Vector(1.0, 2.0, 3.0)
        with pytest.raises(ValueError):
            VectorOperations.divide(v, 0.0)