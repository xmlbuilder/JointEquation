# Block System for Hierarchical Geometry Assembly

A lightweight C++ framework for defining and instancing geometric blocks with hierarchical transformations and bounding box evaluation.

## 🚀 Features
- **Block Definition (`ON_BlockDef`)**
  - Stores name, base point, and geometry list
- **Block Reference (`ON_BlockRef`)**
  - Represents an instance of a block definition
  - Supports local and world transformations
  - Allows hierarchical child references
  - Computes tight bounding boxes in world space
  - Uses dirty flag for efficient matrix updates

## 🧱 Class Overview

### `ON_BlockDef`
```cpp
class ON_BlockDef : public ON_Object {
public:
  ON_BlockDef(const ON_wString& name, const ON_3dPoint& base);
  void AddGeometry(std::shared_ptr<ON_Geometry> g);
  const ON_wString& Name() const;
  const ON_3dPoint& BasePoint() const;
  const std::vector<std::shared_ptr<ON_Geometry>>& Geometries() const;
};
```

### `ON_BlockRef`
```cpp
class ON_BlockRef {
public:
  explicit ON_BlockRef(std::shared_ptr<ON_BlockDef> def);

  // Hierarchy
  void AddChild(std::shared_ptr<ON_BlockRef> c);

  // Local transform
  void SetPivot(const ON_3dPoint& p);
  void SetLocalXform(const ON_Xform& xf);
  void TranslateLocal(double dx, double dy, double dz);
  void RotateAboutPivot(double radians, const ON_3dVector& axis_unit);

  // World transform
  void UpdateMatrices(const ON_Xform& parent_world = ON_Xform::IdentityTransformation);
  const ON_Xform& WorldXform() const;

  // Bounding box
  bool GetTightBoundingBox(ON_BoundingBox& out) const;

  // Access
  const std::shared_ptr<ON_BlockDef>& Def() const;
  const std::vector<std::shared_ptr<ON_BlockRef>>& Children() const;

private:
  std::shared_ptr<ON_BlockDef> m_def;
  ON_Xform m_local = ON_Xform::IdentityTransformation;
  ON_3dPoint m_pivot = ON_3dPoint::Origin;

  mutable ON_Xform m_world = ON_Xform::IdentityTransformation;
  mutable ON_Xform m_parent = ON_Xform::IdentityTransformation;
  mutable bool m_dirty = true;

  std::vector<std::shared_ptr<ON_BlockRef>> m_children;
};
```

## 🔄 Transformation Flow
```
[Parent World] → [Local Transform] → [World Transform]
```
- `UpdateMatrices()` propagates world transforms recursively
- `dirty` flag ensures updates only occur when needed

## 📐 Bounding Box Evaluation
```cpp
bool ON_BlockRef::GetTightBoundingBox(ON_BoundingBox& out) const;
```
- Recursively computes bounding box in world space
- Includes geometry from definition and all children
- Automatically updates matrices if dirty

## 🧠 Design Highlights
- Separation of Definition and Instance
- Hierarchical Transform Propagation
- Efficient Bounding Box Calculation
- Use of `mutable` for internal state tracking
- Minimal dependencies (**OpenNURBS** only)
