## ConvexRegion2d.h

```cpp
template <typename SCALAR>
class ConvexRegion2d {
 public:
  explicit ConvexRegion2d(size_t num_points) {
    this->a_ = matrix_temp<SCALAR>::Zero(num_points, 2);//The matrix's dimensions are num_points x 2 and are initialized to zero
    this->b_ = vector_temp<SCALAR>::Zero(num_points);//The vector's dimensions are num_points x 1 and are initialized to zero
  }
  matrix_temp<SCALAR> getA() const { return a_; }
  vector_temp<SCALAR> getB() const { return b_; }

 protected:
  matrix_temp<SCALAR> a_;
  vector_temp<SCALAR> b_;
};
```

`num_points`，`a_`和`b_`代表什么？

## ConvexRegion2d.cpp

```cpp
template <typename SCALAR>
Rectangle2d<SCALAR>::Rectangle2d(const vector_temp<SCALAR>& pose, const vector_temp<SCALAR>& size) : ConvexRegion2d<SCALAR>(4) {
  matrix_temp<SCALAR> rot(2, 2);
  rot << cos(-pose(2)), -sin(-pose(2)), sin(-pose(2)), cos(-pose(2));
  this->a_ << SCALAR(1), SCALAR(0), SCALAR(-1), SCALAR(0), SCALAR(0), SCALAR(1), SCALAR(0), SCALAR(-1);
  this->a_ = this->a_ * rot;
  vector_temp<SCALAR> pos = rot * pose.head(2);
  this->b_ << pos(0) + size(0) / 2, -(pos(0) - size(0) / 2), pos(1) + size(1) / 2, -(pos(1) - size(1) / 2);
}
```

`pose`每项代表什么

pose(0): x of centroid

pose(1): y of centroid

pose(2): the rotate angle $\theta$

`size`的组成？size(0)，size(1)各项是什么

**`b_`这里计算的是什么**

```cpp
template <typename SCALAR>
Vertex2d<SCALAR>::Vertex2d(size_t num_points, const vector_temp<SCALAR>& points) : ConvexRegion2d<SCALAR>(num_points) {
  size_t size_points = points.size() / 2;  // The numPoints actually greater or equal the size of points/2
  for (size_t i = 0; i < size_points; i++) {
    size_t j = i + 1;
    if (j == size_points) j = 0;
    size_t k = j + 1;
    if (k == size_points) k = 0;
    vector_temp<SCALAR> point_a = points.segment(i * 2, 2);
    vector_temp<SCALAR> point_b = points.segment(j * 2, 2);
    vector_temp<SCALAR> point_c = points.segment(k * 2, 2);

    this->a_.row(i) << point_b.y() - point_a.y(), point_a.x() - point_b.x();
    this->b_(i) = point_a.x() * point_b.y() - point_a.y() * point_b.x();

    if (typeid(SCALAR) == typeid(ad_scalar_t)) {
      SCALAR scalar;
      scalar = CppAD::CondExpGt((this->a_.row(i) * point_c)(0), this->b_(i), SCALAR(-1), SCALAR(1));
      this->a_.row(i) *= scalar;
      this->b_(i) *= scalar;
    } else {
      if ((this->a_.row(i) * point_c)(0) > this->b_(i)) {
        this->a_.row(i) *= SCALAR(-1);
        this->b_(i) *= SCALAR(-1);
      }
    }
  }
}
```

这一段代码在干什么？