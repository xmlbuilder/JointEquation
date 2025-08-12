#pragma once

class ON_CLASS ON_BlockDef : public ON_Object
{
public:
  ON_BlockDef() = default;

  ON_BlockDef(const ON_wString& name, const ON_3dPoint& base)
    : m_name(name), m_base(base) {}

  void AddGeometry(std::shared_ptr<ON_Geometry> g) 
  {
    if (g) m_geoms.push_back(std::move(g));
  }

  const ON_wString& Name() const 
    { return m_name; }

  const ON_3dPoint& BasePoint() const 
    { return m_base; }

  const std::vector<std::shared_ptr<ON_Geometry>>& Geometries() const 
    { return m_geoms; }

private:
  ON_wString m_name;
  ON_3dPoint m_base = ON_3dPoint::Origin;
  std::vector<std::shared_ptr<ON_Geometry>> m_geoms;
};

class ON_CLASS ON_BlockRef
{
public:
  explicit ON_BlockRef(std::shared_ptr<ON_BlockDef> def);

  // 구조
  void AddChild(std::shared_ptr<ON_BlockRef> c);

  // 로컬 상태(보존 대상)
  void SetPivot(const ON_3dPoint& p);
  const ON_3dPoint& Pivot() const;

  void SetLocalXform(const ON_Xform& xf);       // 통째로 설정
  const ON_Xform& LocalXform() const;

  void TranslateLocal(double dx, double dy, double dz);
  void RotateAboutPivot(double radians, const ON_3dVector& axis_unit);

  // 월드 상태(파생값) — UpdateMatrices에서만 갱신
  void UpdateMatrices(const ON_Xform& parent_world = ON_Xform::IdentityTransformation);
  const ON_Xform& WorldXform() const;

  bool GetTightBoundingBoxAccum(ON_BoundingBox& out,
    const ON_Xform& parent_world) const;

  // 쿼리
  bool GetTightBoundingBox(ON_BoundingBox& out) const;

  // 접근
  const std::shared_ptr<ON_BlockDef>& Def() const 
    { return m_def; }

  const std::vector<std::shared_ptr<ON_BlockRef>>& Children() const 
    { return m_children; }

private:
  std::shared_ptr<ON_BlockDef> m_def;

  // 보존 필드
  ON_Xform   m_local = ON_Xform::IdentityTransformation;
  ON_3dPoint m_pivot = ON_3dPoint::Origin;

  // 파생 필드
  ON_Xform  m_world = ON_Xform::IdentityTransformation;
  ON_Xform  m_parent = ON_Xform::IdentityTransformation;

  std::vector<std::shared_ptr<ON_BlockRef>> m_children;

  mutable bool m_dirty = true;

};