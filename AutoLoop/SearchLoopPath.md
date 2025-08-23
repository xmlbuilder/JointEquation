# Parent–Child 그래프에서 루프(사이클)를 피하며 Root→Leaf 경로 자동 탐색
> **Boost.Graph**를 이용해 **Parent/Child만 정의된 유향 그래프**에서
> - 사이클(루프)을 피하고
> - 유효한 Root→Leaf 경로만 수집하며
> - (옵션) 누락된 노드를 `root`까지 부모 방향으로 복원하는
> 간단하고 실용적인 샘플 모음입니다.

---

## 📌 요약
- **문제**: Parent→Child 간선만 있는 DAG가 *아닐 수 있음* (즉, **사이클** 존재 가능).
- **목표**: Root에서 시작해 **루프를 피하면서** 모든 유효한 경로를 수집.
- **추가**: 중복 종단(leaf) 노드 제거, 누락된 노드의 **부모 추적 복원** 유틸 포함.
- **도구**: [Boost Graph Library](https://www.boost.org/doc/libs/release/libs/graph/).

---

## 🧭 목차
1. [아이디어](#아이디어)
2. [빠른 시작 (빌드/실행)](#빠른-시작-빌드실행)
3. [코드: 공통 타입 정의](#코드-공통-타입-정의)
4. [코드: 루프 방지 Root→Leaf 경로 탐색](#코드-루프-방지-rootleaf-경로-탐색)
5. [코드: 종단 중복 제거 및 누락 노드 복원 (bidirectional)](#코드-종단-중복-제거-및-누락-노드-복원-bidirectional)
6. [코드: directed 그래프용 간단 버전](#코드-directed-그래프용-간단-버전)
7. [동작 예시 출력](#동작-예시-출력)
8. [복잡도 & 주의점](#복잡도--주의점)
9. [표: 함수 요약](#표-함수-요약)
10. [Mermaid 흐름도](#mermaid-흐름도)
11. [라이선스](#라이선스)

---

## 아이디어
- DFS(깊이우선탐색) 중 **현재 `path`에 이미 방문한 정점이 다시 등장하면** → **사이클**로 간주하고 **가지치기**.
- 더 내려갈 **유효한 자식이 없다면** 그 경로를 **leaf 경로**로 저장.
- 결과 집합에서 **종단 노드가 같은 경로는 하나만 유지**(중복 leaf 제거).
- (선택) 출력에서 **누락된 정점**이 있다면, `in_edges`를 사용해 **root까지 부모 방향으로 추적**하여 보완.

---

## 빠른 시작 (빌드/실행)
### 요구 사항
- C++17 이상 컴파일러
- Boost.Graph

### g++ 예시
```bash
g++ -std=gnu++17 -O2 loop_paths.cpp -o loop_paths
./loop_paths
```

### CMake 스니펫
```cmake
cmake_minimum_required(VERSION 3.16)
project(loop_paths_demo CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED COMPONENTS graph)

add_executable(loop_paths loop_paths.cpp)
target_link_libraries(loop_paths PRIVATE Boost::graph)
```

---

## 코드: 공통 타입 정의
```cpp
#include <iostream>
#include <vector>
#include <set>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>

using namespace boost;

// in_edges 사용 시 bidirectionalS가 필요합니다.
using Graph  = adjacency_list<vecS, vecS, bidirectionalS>;
using Vertex = graph_traits<Graph>::vertex_descriptor;
```

---

## 코드: 루프 방지 Root→Leaf 경로 탐색
```cpp
// 현재 path 안에 동일 정점이 다시 등장하면 사이클이므로 중단
static inline bool contains(const std::vector<Vertex>& path, Vertex v) {
    return std::find(path.begin(), path.end(), v) != path.end();
}

// 현재 정점에서 "아직 path에 없는" 자식이 하나라도 있으면 true
bool has_valid_child(const Graph& g, Vertex current, const std::vector<Vertex>& path) {
    auto [ei, ei_end] = out_edges(current, g);
    for (; ei != ei_end; ++ei) {
        Vertex next = target(*ei, g);
        if (!contains(path, next)) return true;
    }
    return false;
}

// DFS: 루프를 피하며 root→leaf 경로를 모두 수집
void find_paths(const Graph& g, Vertex current,
                std::vector<Vertex>& path,
                std::vector<std::vector<Vertex>>& all_paths) {
    if (contains(path, current)) return; // 루프 방지
    path.push_back(current);

    if (!has_valid_child(g, current, path)) {
        // 더 진행할 수 없으면 leaf 경로로 저장
        all_paths.push_back(path);
    } else {
        auto [ei, ei_end] = out_edges(current, g);
        for (; ei != ei_end; ++ei) {
            Vertex next = target(*ei, g);
            if (!contains(path, next)) {
                find_paths(g, next, path, all_paths);
            }
        }
    }

    path.pop_back(); // 백트래킹
}
```

---

## 코드: 종단 중복 제거 및 누락 노드 복원 (bidirectional)
- 동일한 **종단(leaf) 정점**을 갖는 경로는 **하나만 출력**합니다.
- **출력에 포함되지 않은 정점**은 `in_edges`로 **부모를 추적**하여 `root`부터의 경로를 한 번 더 출력합니다.

```cpp
// root까지 부모를 따라 경로 복원 (bidirectionalS 필요)
std::vector<Vertex> trace_to_root(const Graph& g, Vertex v, Vertex root) {
    std::vector<Vertex> path;
    std::set<Vertex>   visited; // 한 번 따라간 부모는 중복 방지 (간단한 루프 억제)

    while (true) {
        path.push_back(v);
        if (v == root) break;

        auto [in_begin, in_end] = in_edges(v, g);
        bool found = false;
        for (; in_begin != in_end; ++in_begin) {
            Vertex parent = source(*in_begin, g);
            if (visited.count(parent)) continue;
            visited.insert(parent);
            v = parent;
            found = true;
            break;
        }
        if (!found) break; // root까지 도달하지 못함
    }

    std::reverse(path.begin(), path.end());
    return path;
}

int main() {
    Graph g(10);
    add_edge(0, 1, g);
    add_edge(0, 2, g);
    add_edge(1, 3, g);
    add_edge(1, 4, g);
    add_edge(4, 3, g);
    add_edge(2, 5, g);
    add_edge(5, 6, g);
    add_edge(6, 2, g); // 사이클: 2→5→6→2
    add_edge(2, 8, g);
    add_edge(5, 9, g);

    std::vector<std::vector<Vertex>> all_paths;
    std::vector<Vertex> path;
    find_paths(g, 0, path, all_paths); // root = 0

    // 1) 종단 중복 제거 출력
    std::set<Vertex> printed_ends;
    std::set<Vertex> printed_nodes;
    std::cout << "중복 종단 노드 제거 후 경로:\n";
    for (const auto& p : all_paths) {
        Vertex end = p.back();
        if (printed_ends.count(end)) continue;
        printed_ends.insert(end);
        for (Vertex v : p) {
            std::cout << v << " ";
            printed_nodes.insert(v);
        }
        std::cout << "\n";
    }

    // 2) 누락 노드 복원 출력
    std::cout << "\n누락된 노드 보완 경로:\n";
    Vertex root = 0;
    for (Vertex v = 0; v < num_vertices(g); ++v) {
        if (printed_nodes.count(v)) continue; // 이미 찍은 노드는 생략
        auto recovered = trace_to_root(g, v, root);
        if (!recovered.empty() && recovered.front() == root) {
            bool already_printed = false;
            for (Vertex x : recovered) {
                if (printed_ends.count(x)) { // 종단 중복 회피
                    already_printed = true;
                    break;
                }
            }
            if (!already_printed) {
                for (Vertex x : recovered) {
                    std::cout << x << " ";
                    printed_nodes.insert(x);
                }
                std::cout << "\n";
            }
        }
    }
    return 0;
}
```

---

## 코드: `directed` 그래프용 간단 버전
`in_edges`가 필요 없고, **경로 수집만** 필요한 경우 더 단순화할 수 있습니다.

```cpp
#include <iostream>
#include <vector>
#include <set>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
using namespace boost;

using Graph  = adjacency_list<vecS, vecS, directedS>;
using Vertex = graph_traits<Graph>::vertex_descriptor;

static inline bool contains(const std::vector<Vertex>& path, Vertex v) {
    return std::find(path.begin(), path.end(), v) != path.end();
}

void find_paths(const Graph& g, Vertex current,
                std::vector<Vertex>& path,
                std::vector<std::vector<Vertex>>& all_paths) {
    if (contains(path, current)) return; // 루프 방지
    path.push_back(current);
    auto [ei, ei_end] = out_edges(current, g);
    bool has_unvisited_child = false;
    for (auto it = ei; it != ei_end; ++it) {
        Vertex next = target(*it, g);
        if (!contains(path, next)) {
            has_unvisited_child = true;
            find_paths(g, next, path, all_paths);
        }
    }
    if (!has_unvisited_child) {
        all_paths.push_back(path); // leaf처럼 간주
    }
    path.pop_back();
}

int main() {
    Graph g(10);
    add_edge(0, 1, g);
    add_edge(0, 2, g);
    add_edge(1, 3, g);
    add_edge(1, 4, g);
    add_edge(4, 3, g);
    add_edge(2, 5, g);
    add_edge(5, 6, g);
    add_edge(6, 2, g); // 사이클
    add_edge(2, 8, g);
    add_edge(5, 9, g);

    std::vector<std::vector<Vertex>> all_paths;
    std::vector<Vertex> path;
    find_paths(g, 0, path, all_paths);

    // 종단 중복 제거 출력
    std::set<Vertex> printed_ends;
    std::cout << "중복 종단 노드 제거 후 경로:\n";
    for (const auto& p : all_paths) {
        Vertex end = p.back();
        if (printed_ends.count(end)) continue;
        printed_ends.insert(end);
        for (Vertex v : p) std::cout << v << " ";
        std::cout << "\n";
    }
    return 0;
}
```

---

## 동작 예시 출력
```
중복 종단 노드 제거 후 경로:
0 1 3
0 2 5 6
0 2 5 9
0 2 8

누락된 노드 보완 경로:
0 1 4
```

---

## 복잡도 & 주의점
- **시간복잡도**: 최악의 경우 DFS 경로 수는 **지수적**일 수 있음.
- **공간복잡도**: 재귀 깊이 `O(V)`, 경로 저장량은 경우에 따라 커질 수 있음.
- **주의**:
  - 사이클이 **여러 개** 얽힌 그래프에서는 경로 수가 폭증할 수 있으므로,
    - 깊이 제한, 방문 카운트 제한, 경로 길이 제한 등의 **안전장치**를 고려하세요.
  - `trace_to_root`는 가장 단순한 **한 단계 부모 선택** 버전입니다.
    - 여러 부모 중 우선순위가 있다면 규칙을 더해 주세요 (예: 간선 가중치, 레벨, 시간 등).

---

## 표: 함수 요약

| 함수 | 목적 | 주요 포인트 |
|---|---|---|
| `find_paths` | 루프를 피하며 root→leaf 모든 경로 수집 | `path` 내 재등장 정점은 컷. 유효한 자식 없으면 leaf로 저장 |
| `has_valid_child` | 아직 방문하지 않은 자식 존재 여부 | DFS 가지치기 전 빠른 확인 |
| `trace_to_root` | 누락 노드의 부모 방향 경로 복원 | `in_edges` 필요. 간단한 루프 방지용 `visited` 사용 |

---

## Mermaid 흐름도
```mermaid
flowchart TD
  A[Start at root] --> B{Is current in path?}
  B --> C[Return (cycle)]
  B --> D[Add current to path]
  C:::yes
  D:::no
  D --> E{Has unvisited child?}
  E --> F[Save path as leaf]
  E --> G[DFS on each child not in path]
  F --> H[Remove current (backtrack)]
  G --> H
  H --> I[Done]

  classDef yes fill=#fdd;
  classDef no fill=#dfd;

```


## 전체 소스 참조
```cpp

#include <iostream>
#include <vector>
#include <set>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
using namespace boost;
using Graph = adjacency_list<vecS, vecS, bidirectionalS>; // in_edges 사용을 위해 bidirectionalS
using Vertex = graph_traits<Graph>::vertex_descriptor;
// 루프 방지 + 유효한 경로 탐색
void find_paths(const Graph& g, Vertex current,
                std::vector<Vertex>& path,
                std::vector<std::vector<Vertex>>& all_paths) {
    if (std::find(path.begin(), path.end(), current) != path.end()) return;
    path.push_back(current);
    auto [ei, ei_end] = out_edges(current, g);
    bool has_unvisited_child = false;
    for (auto it = ei; it != ei_end; ++it) {
        Vertex next = target(*it, g);
        if (std::find(path.begin(), path.end(), next) == path.end()) {
            has_unvisited_child = true;
            find_paths(g, next, path, all_paths);
        }
    }
    if (!has_unvisited_child) {
        all_paths.push_back(path);
    }
    path.pop_back();
}
// root까지 부모를 따라 경로 복원
std::vector<Vertex> trace_to_root(const Graph& g, Vertex v, Vertex root) {
    std::vector<Vertex> path;
    std::set<Vertex> visited;
    while (true) {
        path.push_back(v);
        if (v == root) break;
        auto [in_begin, in_end] = in_edges(v, g);
        bool found = false;
        for (; in_begin != in_end; ++in_begin) {
            Vertex parent = source(*in_begin, g);
            if (visited.count(parent)) continue;
            visited.insert(parent);
            v = parent;
            found = true;
            break;
        }
        if (!found) break; // root에 도달하지 못함
    }
    std::reverse(path.begin(), path.end());
    return path;
}
int main() {
    Graph g(10);
    add_edge(0, 1, g);
    add_edge(0, 2, g);
    add_edge(1, 3, g);
    add_edge(1, 4, g);
    add_edge(4, 3, g);
    add_edge(2, 5, g);
    add_edge(5, 6, g);
    add_edge(6, 2, g); // 루프
    add_edge(2, 8, g);
    add_edge(5, 9, g);
    std::vector<std::vector<Vertex>> all_paths;
    std::vector<Vertex> path;
    find_paths(g, 0, path, all_paths);
    std::set<Vertex> printed_ends;
    std::set<Vertex> printed_nodes;
    std::cout << "중복 종단 노드 제거 후 경로:\n";
    for (const auto& p : all_paths) {
        Vertex end = p.back();
        if (printed_ends.count(end)) continue;
        printed_ends.insert(end);
        for (Vertex v : p) {
            std::cout << v << " ";
            printed_nodes.insert(v);
        }
        std::cout << std::endl;
    }
    // 누락된 노드 보완
    std::cout << "\n누락된 노드 보완 경로:\n";
    Vertex root = 0;
    for (Vertex v = 0; v < num_vertices(g); ++v) {
        if (printed_nodes.count(v)) continue;
        std::vector<Vertex> recovered = trace_to_root(g, v, root);
        if (!recovered.empty() && recovered.front() == root) {
            bool already_printed = false;
            for (Vertex x : recovered) {
                if (printed_ends.count(x)) {
                    already_printed = true;
                    break;
                }
            }
            if (!already_printed) {
                for (Vertex x : recovered) {
                    std::cout << x << " ";
                    printed_nodes.insert(x);
                }
                std::cout << std::endl;
            }
        }
    }
    return 0;
}



// 중복 종단 노드 제거 후 경로:
// 0 1 3
// 0 2 5 6
// 0 2 5 9
// 0 2 8
// 누락된 노드 보완 경로:
// 0 1 4













#include <iostream>
#include <vector>
#include <set>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
using namespace boost;
using Graph = adjacency_list<vecS, vecS, directedS>;
using Vertex = graph_traits<Graph>::vertex_descriptor;
// 루프 방지 + 유효한 경로 탐색
void find_paths(const Graph& g, Vertex current,
                std::vector<Vertex>& path,
                std::vector<std::vector<Vertex>>& all_paths) {
    if (std::find(path.begin(), path.end(), current) != path.end()) return; // 루프 방지
    path.push_back(current);
    auto [ei, ei_end] = out_edges(current, g);
    bool has_unvisited_child = false;
    for (auto it = ei; it != ei_end; ++it) {
        Vertex next = target(*it, g);
        if (std::find(path.begin(), path.end(), next) == path.end()) {
            has_unvisited_child = true;
            find_paths(g, next, path, all_paths);
        }
    }
    if (!has_unvisited_child) {
        all_paths.push_back(path); // 더 이상 진행할 수 없는 경우 저장
    }
    path.pop_back();
}
int main() {
    Graph g(10);
    add_edge(0, 1, g);
    add_edge(0, 2, g);
    add_edge(1, 3, g);
    add_edge(1, 4, g);
    add_edge(4, 3, g);
    add_edge(2, 5, g);
    add_edge(5, 6, g);
    add_edge(6, 2, g); // 루프: 2 → 5 → 6 → 2
    add_edge(2, 8, g);
    add_edge(5, 9, g);
    std::vector<std::vector<Vertex>> all_paths;
    std::vector<Vertex> path;
    find_paths(g, 0, path, all_paths);
    std::set<Vertex> printed_ends;
    std::cout << "중복 종단 노드 제거 후 경로:\n";
    for (const auto& p : all_paths) {
        Vertex end = p.back();
        if (printed_ends.count(end)) continue;
        printed_ends.insert(end);
        for (Vertex v : p)
            std::cout << v << " ";
        std::cout << std::endl;
    }
    return 0;
}


// 중복 종단 노드 제거 후 경로:
// 0 1 3
// 0 2 5 6
// 0 2 5 9
// 0 2 8







#include <iostream>
#include <vector>
#include <boost/graph/adjacency_list.hpp>
using namespace boost;
using Graph = adjacency_list<vecS, vecS, directedS>;
using Vertex = graph_traits<Graph>::vertex_descriptor;
// 유효한 자식이 있는지 확인 (루프 제외)
bool has_valid_child(const Graph& g, Vertex current, const std::vector<Vertex>& path) {
    auto [ei, ei_end] = out_edges(current, g);
    for (; ei != ei_end; ++ei) {
        Vertex next = target(*ei, g);
        if (std::find(path.begin(), path.end(), next) == path.end())
            return true;
    }
    return false;
}
// 루프를 피하면서 모든 root-to-leaf 경로 탐색
void find_paths(const Graph& g, Vertex current,
                std::vector<Vertex>& path,
                std::vector<std::vector<Vertex>>& all_paths) {
    if (std::find(path.begin(), path.end(), current) != path.end()) return; // 루프 방지
    path.push_back(current);
    if (!has_valid_child(g, current, path)) {
        all_paths.push_back(path); // leaf처럼 간주
    } else {
        auto [ei, ei_end] = out_edges(current, g);
        for (; ei != ei_end; ++ei) {
            Vertex next = target(*ei, g);
            if (std::find(path.begin(), path.end(), next) == path.end()) {
                find_paths(g, next, path, all_paths);
            }
        }
    }
    path.pop_back(); // 백트래킹
}
int main() {
    Graph g(7);
    add_edge(0, 1, g);
    add_edge(0, 2, g);
    add_edge(1, 3, g);
    add_edge(1, 4, g);
    add_edge(4, 3, g);
    add_edge(2, 5, g);
    add_edge(5, 6, g);
    add_edge(6, 2, g); // 루프: 2 → 5 → 6 → 2
    add_edge(2, 8, g);
    add_edge(5, 9, g);
    std::vector<std::vector<Vertex>> all_paths;
    std::vector<Vertex> path;
    find_paths(g, 0, path, all_paths); // root = 0
    std::cout << "모든 root-to-leaf 경로 (루프 제거):\n";
    for (const auto& p : all_paths) {
        for (Vertex v : p)
            std::cout << v << " ";
        std::cout << std::endl;
    }
    return 0;
}

// 모든 root-to-leaf 경로 (루프 제거):
// 0 1 3
// 0 1 4 3
// 0 2 5 6
// 0 2 5 9
// 0 2 8



```

---

## 라이선스
원하는 라이선스를 추가하세요. 예) MIT, Apache-2.0 등.

---

## 부록: 작은 팁
- 정점 ID가 **희소**하거나 **큰 정수**일 수 있다면 `std::vector<bool>` 대신 `std::unordered_set<Vertex>`로 방문 체크를 바꾸는 것도 실전에서 편합니다.
- `boost::filtered_graph`로 **사전 필터**를 걸어 불필요한 간선을 제거하면 경로 수가 크게 줄어들 수 있습니다.
