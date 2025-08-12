#include "opennurbs.h"

#include <iterator>

ON_BlockRef::ON_BlockRef(std::shared_ptr<ON_BlockDef> def)
  : m_def(std::move(def))
{
  // m_local, m_world, m_pivot은 멤버 기본값 그대로
}

void ON_BlockRef::AddChild(std::shared_ptr<ON_BlockRef> c)
{
  if (c) m_children.push_back(std::move(c));
}

void ON_BlockRef::SetPivot(const ON_3dPoint& p) { m_pivot = p; }
const ON_3dPoint& ON_BlockRef::Pivot() const { return m_pivot; }

void ON_BlockRef::SetLocalXform(const ON_Xform& xf) { 
  m_local = xf;
  m_dirty = true;
}
const ON_Xform& ON_BlockRef::LocalXform() const { return m_local; }

void ON_BlockRef::TranslateLocal(double dx, double dy, double dz)
{
  m_local = ON_Xform::TranslationTransformation(ON_3dVector(dx, dy, dz)) 
    * m_local;

  m_dirty = true;


}

void ON_BlockRef::RotateAboutPivot(
  double radians, 
  const ON_3dVector& axis_unit)
{
  ON_Xform R;
  R.Rotation(radians, axis_unit, m_pivot);
  m_local = R * m_local;
  m_dirty = true;
}

void ON_BlockRef::UpdateMatrices(const ON_Xform& parent_world)
{
  if (!m_dirty && m_parent == parent_world)
    return; // 이미 최신 상태


  // ★ 유일한 누적 지점: 부모 * 로컬
  m_world = m_local * parent_world;
  m_parent = parent_world;
  m_dirty = false;


  // 자식으로 전파
  for (auto& c : m_children)
    c->UpdateMatrices(m_world);
}

const ON_Xform& ON_BlockRef::WorldXform() const 
{ 
  return m_world; 
}

bool ON_BlockRef::GetTightBoundingBoxAccum(
  ON_BoundingBox& out,
  const ON_Xform& parent_world) const
{
  bool ok = false;

  // 1) 나의 world = m_local * parent_world (행렬 순서 주의)
   ON_Xform this_world = m_local * parent_world;

  // 2) 내 정의 지오메트리들: m_world 대신 this_world 사용
  if (m_def)
  {
    for (const auto& g : m_def->Geometries())
    {
      if (!g) continue;
      ON_BoundingBox gb;
      if (g->GetTightBoundingBox(gb, false, &this_world)) {
        out.Union(gb);
        ok = true;
      }
    }
  }

  // 3) 자식들: this_world를 부모로 내려보냄
  for (const auto& c : m_children)
  {
    if (!c) continue;
    ON_BoundingBox cb;
    if (c->GetTightBoundingBoxAccum(cb, this_world)) {
      out.Union(cb);
      ok = true;
    }
  }
  return ok;
}

// 루트 진입점 헬퍼 (아이덴티티에서 시작)
bool ON_BlockRef::GetTightBoundingBox(
  ON_BoundingBox& out) const
{
  out.Destroy(); // 무효로 초기화

  if (m_dirty)
    const_cast<ON_BlockRef*>(this)->UpdateMatrices(m_parent);

  return GetTightBoundingBoxAccum(out, m_parent);
}

