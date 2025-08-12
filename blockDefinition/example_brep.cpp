#include "../opennurbs_public_examples.h"
#include <iostream>
#include <limits>

int main(int argc, const char* argv[])
{
  ON::Begin();
  ON_TextLog log;

  auto defE0 = std::make_shared<ON_BlockDef>(L"E0", ON_3dPoint::Origin);
  auto defE1 = std::make_shared<ON_BlockDef>(L"E1", ON_3dPoint::Origin);
  auto defE2 = std::make_shared<ON_BlockDef>(L"E2", ON_3dPoint::Origin);
  auto defE3 = std::make_shared<ON_BlockDef>(L"E3", ON_3dPoint::Origin);
  auto defE4 = std::make_shared<ON_BlockDef>(L"E4", ON_3dPoint::Origin);
  auto defE5 = std::make_shared<ON_BlockDef>(L"E5", ON_3dPoint::Origin);
  auto defE6 = std::make_shared<ON_BlockDef>(L"E6", ON_3dPoint::Origin);

  defE0->AddGeometry(std::shared_ptr<ON_Geometry>(new ON_LineCurve(ON_3dPoint(0, 0, 0), ON_3dPoint(0, 0, 80))));
  defE1->AddGeometry(std::shared_ptr<ON_Geometry>(new ON_LineCurve(ON_3dPoint(0, 0, 0), ON_3dPoint(0, 0, 90))));
  defE2->AddGeometry(std::shared_ptr<ON_Geometry>(new ON_LineCurve(ON_3dPoint(0, 0, 0), ON_3dPoint(0, 0, 100))));
  defE3->AddGeometry(std::shared_ptr<ON_Geometry>(new ON_LineCurve(ON_3dPoint(0, 0, 0), ON_3dPoint(0, 0, 110))));
  defE4->AddGeometry(std::shared_ptr<ON_Geometry>(new ON_LineCurve(ON_3dPoint(0, 0, 0), ON_3dPoint(0, 0, 120))));
  defE5->AddGeometry(std::shared_ptr<ON_Geometry>(new ON_LineCurve(ON_3dPoint(0, 0, 0), ON_3dPoint(0, 0, 130))));
  defE6->AddGeometry(std::shared_ptr<ON_Geometry>(new ON_LineCurve(ON_3dPoint(0, 0, 0), ON_3dPoint(0, 0, 140))));

  auto r6 = std::make_shared<ON_BlockRef>(defE6); 
  r6->SetPivot({ 0,0,0 });

  auto r5 = std::make_shared<ON_BlockRef>(defE5); 
  r5->SetPivot({ 0,0,0 });
  r5->AddChild(r6);

  auto r4 = std::make_shared<ON_BlockRef>(defE4); 
  r4->AddChild(r5);
  auto r3 = std::make_shared<ON_BlockRef>(defE3); 
  r3->AddChild(r4);
  auto r2 = std::make_shared<ON_BlockRef>(defE2); 
  r2->AddChild(r3);
  auto r1 = std::make_shared<ON_BlockRef>(defE1); 
  r1->AddChild(r2);
  auto root = std::make_shared<ON_BlockRef>(defE0); 
  root->AddChild(r1);

  // 링크 연결(이전 링크 끝으로 이동) — 높이는 위에서 정의한 h 값 사용
  r2->TranslateLocal(0, 0, 90);
  r3->TranslateLocal(0, 0, 100);
  r4->TranslateLocal(0, 0, 100);
  r5->TranslateLocal(0, 0, 120);
  r6->TranslateLocal(0, 0, 120);

  // 조인트 회전(로컬만 수정, 보존됨)
  root->RotateAboutPivot(ON_RadiansFromDegrees(90), ON_3dVector::YAxis);
  //r2->RotateAboutPivot(ON_RadiansFromDegrees(10), ON_3dVector::YAxis);

  // ★ 월드 누적
  root->UpdateMatrices();

  // ★ BB 쿼리
  ON_BoundingBox bb;
  if (r6->GetTightBoundingBox(bb)) {
    std::wprintf(L"Robot BB: min(%.1f,%.1f,%.1f) max(%.1f,%.1f,%.1f)\n",
      bb.m_min.x, bb.m_min.y, bb.m_min.z,
      bb.m_max.x, bb.m_max.y, bb.m_max.z);
  }
  ON::End();
  return 0;
}

