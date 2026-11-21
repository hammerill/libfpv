// libfpv - the overlay's collision view (collision_view.h).
//
// Drawn with Dear ImGui's 2D triangles into the background draw list, so
// every game that draws the overlay shows it without a line of its own:
// libfpv projects the collision world through the camera of the view the
// game shows (the picture's rectangle and the fisheye as the video
// post-process draws them), and sorts the triangles back to front, as there
// is no depth buffer. The nearest m_budget triangles within m_distance are
// drawn, so a large level stays fast enough to fly in.
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#include "overlay/collision_view.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <string>

#include "imgui.h"
#include "imgui_internal.h"  // ImDrawListSharedData: the white pixel for PrimWriteVtx
#include "lib.h"
#include "util/platform.h"

namespace fpv {

namespace {

constexpr double kImpactShowTime = 2.0;  // s an impact stays on screen
constexpr size_t kCollectLimit = 400000;  // triangles gathered near the camera at most
constexpr double kLegendTime = 10.0;      // s the legend shows after choosing a view
constexpr int kMaxSplit = 10;             // levels a big triangle is cut into (4 pieces each)
constexpr float kMinPiece = 0.25f;        // m, pieces are not cut smaller

struct Rgb {
    float r, g, b;
};
uint32_t col(Rgb c, float a) {
    auto q = [](float v) { return (int)std::clamp(v * 255.0f + 0.5f, 0.0f, 255.0f); };
    return IM_COL32(q(c.r), q(c.g), q(c.b), q(a));
}
Rgb mix(Rgb a, Rgb b, float t) { return {a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t}; }
Rgb scale(Rgb a, float s) { return {a.r * s, a.g * s, a.b * s}; }

// the palette (the legend shows the same)
const Rgb kBackground{0.075f, 0.085f, 0.105f};
const Rgb kSurface{0.62f, 0.66f, 0.72f};
const Rgb kObject{0.38f, 0.68f, 0.86f};
const Rgb kKillsProps{0.95f, 0.22f, 0.18f};
const Rgb kVideoClear{0.40f, 0.85f, 0.52f};
const Rgb kNoCollision{0.72f, 0.58f, 0.95f};
const Rgb kPlane{0.98f, 0.80f, 0.25f};
const Rgb kCollider{0.35f, 0.85f, 1.0f};
const Rgb kBody{0.92f, 0.92f, 0.96f};
const Rgb kProp{1.0f, 0.62f, 0.15f};
const Rgb kContact{0.70f, 1.0f, 0.30f};
const Rgb kVtxHit{0.95f, 0.35f, 1.0f};
const Rgb kHit{0.95f, 0.95f, 0.95f};

// A filled triangle; an opaque one grown by about half a pixel, so the
// pieces of a cut triangle meet without cracks showing the background.
// Each corner has its own colour (the fog, by its distance).
void fillTri(ImDrawList *dl, ImVec2 a, ImVec2 b, ImVec2 c, uint32_t ca, uint32_t cb, uint32_t cc, bool opaque) {
    if (opaque) {
        ImVec2 m((a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3);
        auto grow = [&](ImVec2 &p) {
            float dx = p.x - m.x, dy = p.y - m.y, l = std::sqrt(dx * dx + dy * dy);
            if (l > 1e-3f) p.x += dx / l * 0.6f, p.y += dy / l * 0.6f;
        };
        grow(a), grow(b), grow(c);
    }
    const ImVec2 uv = dl->_Data->TexUvWhitePixel;
    dl->PrimReserve(3, 3);
    ImDrawIdx i0 = (ImDrawIdx)dl->_VtxCurrentIdx;
    dl->PrimWriteIdx(i0);
    dl->PrimWriteIdx((ImDrawIdx)(i0 + 1));
    dl->PrimWriteIdx((ImDrawIdx)(i0 + 2));
    dl->PrimWriteVtx(a, uv, ca);
    dl->PrimWriteVtx(b, uv, cb);
    dl->PrimWriteVtx(c, uv, cc);
}

// How far a point at `d` (m) has faded into the background.
float fogAt(float d, float dist) { return 0.8f * std::pow(std::clamp(d / dist, 0.0f, 1.0f), 1.3f); }

Vec3 basisA(Vec3 n) {
    Vec3 a = std::fabs(n.y) < 0.9f ? Vec3{0, 1, 0} : Vec3{1, 0, 0};
    return normalize(cross(a, n), {1, 0, 0});
}

}  // namespace

void CollisionView::Projection::screen(Vec3 v, float &x, float &y) const {
    float z = std::max(v.z, nearZ);
    float dx = v.x / z / tanX * 0.5f, dy = v.y / z / tanY * 0.5f;  // -0.5..0.5 across the picture
    if (lens) {
        // The post-process samples the scene at c (1 + k1 r^2 + k2 r^4) / zoom
        // for the output's c: invert it (monotonic for k1, k2 >= 0)
        float ex = dx * scaleX, ey = dy * scaleY;
        float re = std::sqrt(ex * ex + ey * ey);
        if (re > 1e-6f) {
            float r = re;
            for (int i = 0; i < 8; i++) {
                float r2 = r * r;
                float g = r * (1 + k1 * r2 + k2 * r2 * r2) / zoom - re;
                float dg = (1 + 3 * k1 * r2 + 5 * k2 * r2 * r2) / zoom;
                r = std::max(0.0f, r - g / std::max(dg, 1e-3f));
            }
            dx = ex * (r / re) / scaleX;
            dy = ey * (r / re) / scaleY;
        }
    }
    x = rx + (0.5f + dx) * rw;
    y = ry + (0.5f - dy) * rh;
}

CollisionView::CollisionView() {
    std::string e = envUtf8("LIBFPV_COLLISION_VIEW");
    if (e == "1" || e == "solid") m_mode = Solid;
    if (e == "2" || e == "xray") m_mode = XRay;
}

void CollisionView::ui() {
    const char *modes[] = {"Off", "Instead of the game's picture", "Over the game's picture (x-ray)"};
    if (ImGui::Combo("Show collision", &m_mode, modes, 3) && on()) {
        // choosing a view shows the legend again for a while
        m_legend = true;
        m_legendUntil = -1;
    }
    ImGui::TextDisabled("What the drone collides with, seen from the view the game shows.");
    ImGui::TextDisabled("Red surfaces break the props; hits faster than %.0f m/s damage the VTX.", kVtxHitSpeed);
    if (!on()) return;
    ImGui::Checkbox("The drone's collision spheres and prop discs", &m_colliders);
    ImGui::Checkbox("Contacts and impacts", &m_contacts);
    ImGui::SliderFloat("Draw distance (m)", &m_distance, 10, 2000, "%.0f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderInt("Triangles at most", &m_budget, 2000, 300000, "%d", ImGuiSliderFlags_Logarithmic);
    ImGui::TextDisabled("Shown with this overlay closed too, until set to Off; not saved.");
}

bool CollisionView::setupProjection(float w, float h, Projection &pr) const {
    Lib &L = lib();
    if (!L.lastFrameValid || w <= 0 || h <= 0) return false;
    const fpv_frame &f = L.lastFrame;
    const CoordConverter &cv = L.conv;
    const fpv_camera *cam = &f.camera;
    if (L.view == FPV_VIEW_CHASE) cam = &f.chase_camera;
    if (L.view == FPV_VIEW_LOS || L.view == FPV_VIEW_PILOT) cam = &f.los_camera;
    auto v3 = [](const float *p) { return Vec3{p[0], p[1], p[2]}; };
    pr.pos = cv.posToInternal(v3(cam->position));
    pr.right = normalize(cv.dirToInternal(v3(cam->right)), {1, 0, 0});
    pr.up = normalize(cv.dirToInternal(v3(cam->up)), {0, 1, 0});
    pr.fwd = normalize(cv.dirToInternal(v3(cam->forward)), {0, 0, 1});
    if (L.view == FPV_VIEW_PILOT && L.listenerSet) {
        // the game's own camera, as it gives it for the ear
        pr.pos = L.listenerPos;
        pr.right = rotate(L.listenerRot, {1, 0, 0});
        pr.up = rotate(L.listenerRot, {0, 1, 0});
        pr.fwd = rotate(L.listenerRot, {0, 0, 1});
    }
    pr.nearZ = std::max(0.005f, cv.lenToInternal(cam->near_plane));
    float fov = std::clamp(cam->fov_y_degrees, 1.0f, 175.0f);
    pr.tanY = std::tan(fov * kPi / 360.0f);
    // the picture: all of the screen, or the camera's at its aspect with
    // black bars (the video feed's resolution and aspect, FPV view only)
    const bool fpvView = L.view == FPV_VIEW_FPV;
    float aspect = w / h;
    pr.rx = 0, pr.ry = 0, pr.rw = w, pr.rh = h;
    const fpv_video_feed &vf = f.video_feed;
    if (fpvView && (vf.flags & FPV_VIDEO_FEED_RESOLUTION) && vf.picture_aspect > 0.1f) {
        float shown = vf.display_aspect > 0.1f ? vf.display_aspect : vf.picture_aspect;
        aspect = vf.picture_aspect;
        if (pr.rw / pr.rh > shown) {
            pr.rw = pr.rh * shown;
            pr.rx = (w - pr.rw) * 0.5f;
        } else {
            pr.rh = pr.rw / shown;
            pr.ry = (h - pr.rh) * 0.5f;
        }
    }
    pr.tanX = pr.tanY * aspect;
    pr.k1 = cam->distortion_k1;
    pr.k2 = cam->distortion_k2;
    pr.zoom = cam->distortion_zoom > 0.01f ? cam->distortion_zoom : 1.0f;
    pr.lens = fpvView && !(L.config.flags & FPV_INIT_NO_LENS_DISTORTION) && (pr.k1 != 0 || pr.k2 != 0);
    float s = 0.5f * std::sqrt(aspect * aspect + 1);  // r = 1 at the corners
    pr.scaleX = aspect / s;
    pr.scaleY = 1 / s;
    return true;
}

void CollisionView::draw(ImDrawList *dl, float w, float h, double now) {
    Projection pr;
    bool have = setupProjection(w, h, pr);
    if (m_mode == Solid) {
        dl->AddRectFilled(ImVec2(0, 0), ImVec2(w, h), IM_COL32(0, 0, 0, 255));
        dl->AddRectFilled(ImVec2(pr.rx, pr.ry), ImVec2(pr.rx + pr.rw, pr.ry + pr.rh), col(kBackground, 1));
    }
    if (have) {
        // flat triangles without the anti-aliasing fringe: a third of the vertices
        ImDrawListFlags saved = dl->Flags;
        dl->Flags &= ~ImDrawListFlags_AntiAliasedFill;
        dl->PushClipRect(ImVec2(pr.rx, pr.ry), ImVec2(pr.rx + pr.rw, pr.ry + pr.rh), true);
        drawWorld(dl, pr);
        drawPlanes(dl, pr);
        if (m_colliders || m_contacts) drawDrone(dl, pr);
        if (m_contacts) drawImpacts(dl, pr, now);
        dl->PopClipRect();
        dl->Flags = saved;
    }
    drawLegend(dl, w, h);
}

void CollisionView::drawWorld(ImDrawList *dl, const Projection &pr) {
    Lib &L = lib();
    const float dist = std::max(1.0f, m_distance);
    // the box around the view's frustum out to the draw distance
    Vec3 lo = pr.pos, hi = pr.pos;
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2) {
            Vec3 c = pr.pos + (pr.fwd + pr.right * (sx * pr.tanX) + pr.up * (sy * pr.tanY)) * dist;
            lo = vmin(lo, c);
            hi = vmax(hi, c);
        }
    // a wide FOV's corners reach further out than the distance: keep to it
    Vec3 reach{dist, dist, dist};
    lo = vmax(lo, pr.pos - reach);
    hi = vmin(hi, pr.pos + reach);

    std::lock_guard<std::mutex> lock(L.world.mutex());
    m_tris.clear();
    bool complete = L.world.debugTriangles(lo, hi, L.lastFrame.sim_time, kCollectLimit, m_tris);
    m_noWorld = L.world.staticTriangleCount() == 0 && m_tris.empty();
    m_items.clear();
    m_cut.clear();
    for (uint32_t i = 0; i < m_tris.size(); i++) {
        const auto &t = m_tris[i];
        size_t before = m_items.size();
        addPieces(pr, i, pr.toView(t.a), pr.toView(t.b), pr.toView(t.c), 7, 0);
        if (m_items.size() > before + 1) m_cut.push_back(i);
    }
    m_near = m_items.size();
    m_truncated = !complete;
    size_t budget = (size_t)std::max(1, m_budget);
    if (m_items.size() > budget) {
        std::nth_element(m_items.begin(), m_items.begin() + (std::ptrdiff_t)budget, m_items.end(),
                         [](const Item &x, const Item &y) { return x.depth < y.depth; });
        m_items.resize(budget);
        m_truncated = true;
    }
    std::sort(m_items.begin(), m_items.end(), [](const Item &x, const Item &y) { return x.depth > y.depth; });
    m_drawn = m_items.size();

    const bool xray = m_mode == XRay;
    const Vec3 sun = normalize(Vec3{0.35f, 1.0f, 0.25f});
    auto shade = [&](const CollisionWorld::DebugTriangle &t, float lumScale) {
        Vec3 n = normalize(cross(t.b - t.a, t.c - t.a), pr.up);
        return lumScale * (0.35f + 0.35f * std::fabs(dot(n, pr.fwd)) + 0.3f * std::fabs(dot(n, sun)));
    };
    if (!xray) {
        // under the pieces of a cut triangle, the whole of it: the hairline
        // cracks between thin far pieces show its own colour, not the sky
        for (uint32_t i : m_cut) {
            const auto &t = m_tris[i];
            Material m = L.world.material(t.material);
            if (m.flags & (kMatNoCollision | kMatTransparentToVideo)) continue;
            Rgb base = (m.flags & kMatKillsProps) ? kKillsProps : t.objectId ? kObject : kSurface;
            Vec3 v[3] = {pr.toView(t.a), pr.toView(t.b), pr.toView(t.c)};
            drawTriangle(dl, pr, v, col(scale(base, shade(t, 1)), 1), 0, 1, 0, dist);
        }
    }
    for (const Item &it : m_items) {
        const auto &t = m_tris[it.index];
        Material m = L.world.material(t.material);
        // flat, from the normal alone: a big triangle is one shade, and
        // neighbours facing the same way match
        float lum = shade(t, 1);
        const Vec3 *v = it.v;
        // the outline fades by the nearest corner; the fill fades corner by corner
        float fog = fogAt(std::min({length(v[0]), length(v[1]), length(v[2])}), dist);
        Rgb base = (m.flags & kMatKillsProps) ? kKillsProps
                   : (m.flags & kMatNoCollision) ? kNoCollision
                   : (m.flags & kMatTransparentToVideo) ? kVideoClear
                   : t.objectId ? kObject : kSurface;
        Rgb shaded = scale(base, lum);
        float fillA = xray ? 0.22f : 1.0f, edgeA = xray ? 0.85f : 0.35f;
        Rgb edge = xray ? mix(base, kBackground, fog) : scale(mix(shaded, kBackground, fog), 0.55f);
        if (m.flags & kMatNoCollision) {
            fillA = 0;  // only seen by the video link: an outline
            edgeA = 0.7f;
            edge = mix(base, kBackground, fog);
        } else if (m.flags & kMatTransparentToVideo) {
            fillA *= 0.5f;
        }
        edgeA *= 1 - fog;
        drawTriangle(dl, pr, v, fillA > 0 ? col(shaded, fillA) : 0, col(edge, edgeA), 1.0f, it.edges, dist);
    }
}

void CollisionView::addPieces(const Projection &pr, uint32_t index, Vec3 a, Vec3 b, Vec3 c, unsigned edges,
                              int depth) {
    const float dist = std::max(1.0f, m_distance);
    if (a.z < pr.nearZ && b.z < pr.nearZ && c.z < pr.nearZ) return;
    // the draw distance drops whole triangles only: pieces would leave a ragged edge
    if (depth == 0 && a.z > dist && b.z > dist && c.z > dist) return;
    // all three beyond one side of the frustum
    const float mx = pr.tanX * 1.02f, my = pr.tanY * 1.02f;
    if ((a.x > a.z * mx && b.x > b.z * mx && c.x > c.z * mx) ||
        (-a.x > a.z * mx && -b.x > b.z * mx && -c.x > c.z * mx) ||
        (a.y > a.z * my && b.y > b.z * my && c.y > c.z * my) ||
        (-a.y > a.z * my && -b.y > b.z * my && -c.y > c.z * my))
        return;
    float longest = std::sqrt(std::max({length2(b - a), length2(c - b), length2(a - c)}));
    float away = length(closestPointOnTriangle(Vec3{0, 0, 0}, a, b, c));
    if (depth < kMaxSplit && longest > kMinPiece && longest > away && away < dist) {
        // four pieces at the midpoints; a side stays outlined where it is
        // part of one of the triangle's own
        Vec3 ab = (a + b) * 0.5f, bc = (b + c) * 0.5f, ca = (c + a) * 0.5f;
        addPieces(pr, index, a, ab, ca, (edges & 1) | (edges & 4), depth + 1);
        addPieces(pr, index, ab, b, bc, (edges & 1) | (edges & 2), depth + 1);
        addPieces(pr, index, ca, bc, c, (edges & 2) | (edges & 4), depth + 1);
        addPieces(pr, index, ab, bc, ca, 0, depth + 1);
        return;
    }
    // sorted by the part that is seen: in front of the near plane
    Vec3 v[3] = {a, b, c};
    float sum = 0;
    int n = 0;
    for (int i = 0; i < 3; i++) {
        Vec3 p = v[i], q = v[(i + 1) % 3];
        if (p.z >= pr.nearZ) sum += p.z, n++;
        if ((p.z >= pr.nearZ) != (q.z >= pr.nearZ)) sum += pr.nearZ, n++;
    }
    m_items.push_back({n ? sum / n : pr.nearZ, index, {a, b, c}, edges});
}

void CollisionView::drawTriangle(ImDrawList *dl, const Projection &pr, const Vec3 v[3], uint32_t fill, uint32_t edge,
                                 float edgeWidth, unsigned edges, float fogDist) {
    // near clip (Sutherland-Hodgman against z = nearZ): up to four corners,
    // each marked when the clip made it
    // and the side each starts (the next corner lies along it, unless both
    // come from the clip: then the near plane's own edge)
    Vec3 poly[4];
    bool cut[4];
    int side[4];
    int n = 0;
    for (int i = 0; i < 3; i++) {
        Vec3 a = v[i], b = v[(i + 1) % 3];
        bool ina = a.z >= pr.nearZ, inb = b.z >= pr.nearZ;
        if (ina) poly[n] = a, side[n] = i, cut[n++] = false;
        if (ina != inb) {
            float t = (pr.nearZ - a.z) / (b.z - a.z);
            poly[n] = lerp(a, b, t), side[n] = i, cut[n++] = true;
        }
    }
    if (n < 3) return;
    float sx[4], sy[4];
    float x0 = 1e30f, y0 = 1e30f, x1 = -1e30f, y1 = -1e30f;
    for (int i = 0; i < n; i++) {
        pr.screen(poly[i], sx[i], sy[i]);
        x0 = std::min(x0, sx[i]), x1 = std::max(x1, sx[i]);
        y0 = std::min(y0, sy[i]), y1 = std::max(y1, sy[i]);
    }
    if (x1 < pr.rx || y1 < pr.ry || x0 > pr.rx + pr.rw || y0 > pr.ry + pr.rh) return;
    // through the lens a straight edge bends: big triangles are cut finer
    int level = 0;
    if (pr.lens) {
        float size = std::max(x1 - x0, y1 - y0);
        level = size > 600 ? 4 : size > 200 ? 3 : size > 60 ? 2 : size > 16 ? 1 : 0;
    }
    if (fill & IM_COL32_A_MASK) {
        const bool opaque = (fill & IM_COL32_A_MASK) == IM_COL32_A_MASK;
        const Rgb fc{((fill >> IM_COL32_R_SHIFT) & 255) / 255.0f, ((fill >> IM_COL32_G_SHIFT) & 255) / 255.0f,
                     ((fill >> IM_COL32_B_SHIFT) & 255) / 255.0f};
        const float fa = ((fill >> IM_COL32_A_SHIFT) & 255) / 255.0f;
        auto colourAt = [&](Vec3 p) {
            if (fogDist <= 0) return fill;
            float fog = fogAt(length(p), fogDist);
            return col(mix(fc, kBackground, fog), opaque ? 1.0f : fa * (1 - 0.5f * fog));
        };
        for (int f = 1; f + 1 < n; f++) {
            if (level == 0) {
                fillTri(dl, ImVec2(sx[0], sy[0]), ImVec2(sx[f], sy[f]), ImVec2(sx[f + 1], sy[f + 1]), colourAt(poly[0]),
                        colourAt(poly[f]), colourAt(poly[f + 1]), opaque);
                continue;
            }
            const int k = 1 << level;
            Vec3 A = poly[0], B = poly[f], C = poly[f + 1];
            // the grid's points, row by row (j up, i along)
            ImVec2 pts[17 * 18 / 2];
            uint32_t cols[17 * 18 / 2];
            auto idx = [&](int i, int j) { return j * (2 * k + 3 - j) / 2 + i; };
            for (int j = 0; j <= k; j++)
                for (int i = 0; i + j <= k; i++) {
                    Vec3 p = A + (B - A) * ((float)i / k) + (C - A) * ((float)j / k);
                    ImVec2 &s = pts[idx(i, j)];
                    pr.screen(p, s.x, s.y);
                    cols[idx(i, j)] = colourAt(p);
                }
            auto tri = [&](int a, int b, int c) { fillTri(dl, pts[a], pts[b], pts[c], cols[a], cols[b], cols[c], opaque); };
            for (int j = 0; j < k; j++)
                for (int i = 0; i + j < k; i++) {
                    tri(idx(i, j), idx(i + 1, j), idx(i, j + 1));
                    if (i + j + 1 < k) tri(idx(i + 1, j), idx(i + 1, j + 1), idx(i, j + 1));
                }
        }
    }
    if (edge & IM_COL32_A_MASK) {
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            if (cut[i] && cut[j]) continue;  // the near plane's own edge
            if (!(edges >> side[i] & 1)) continue;  // inside the triangle
            if (level == 0) {
                dl->AddLine(ImVec2(sx[i], sy[i]), ImVec2(sx[j], sy[j]), edge, edgeWidth);
            } else {
                drawSegment(dl, pr, poly[i], poly[j], edge, edgeWidth);
            }
        }
    }
}

void CollisionView::drawSegment(ImDrawList *dl, const Projection &pr, Vec3 a, Vec3 b, uint32_t c, float width) {
    if (a.z < pr.nearZ && b.z < pr.nearZ) return;
    if (a.z < pr.nearZ) a = lerp(a, b, (pr.nearZ - a.z) / (b.z - a.z));
    if (b.z < pr.nearZ) b = lerp(b, a, (pr.nearZ - b.z) / (a.z - b.z));
    const int steps = pr.lens ? 12 : 1;
    ImVec2 pts[13];
    for (int i = 0; i <= steps; i++) pr.screen(lerp(a, b, (float)i / steps), pts[i].x, pts[i].y);
    dl->AddPolyline(pts, steps + 1, c, ImDrawFlags_None, width);
}

void CollisionView::drawPlanes(ImDrawList *dl, const Projection &pr) {
    Lib &L = lib();
    std::lock_guard<std::mutex> lock(L.world.mutex());
    const std::vector<Contact> &planes = L.world.proxyPlanes();
    const float half = 0.3f;
    for (const Contact &p : planes) {
        Material m = L.world.material(p.material);
        Rgb c = (m.flags & kMatKillsProps) ? kKillsProps : kPlane;
        Vec3 a = basisA(p.normal), b = cross(p.normal, a);
        Vec3 q[4] = {p.point + (a + b) * half, p.point + (a - b) * half, p.point + (-a - b) * half,
                     p.point + (-a + b) * half};
        Vec3 v0[3] = {pr.toView(q[0]), pr.toView(q[1]), pr.toView(q[2])};
        Vec3 v1[3] = {pr.toView(q[0]), pr.toView(q[2]), pr.toView(q[3])};
        drawTriangle(dl, pr, v0, col(c, 0.30f), 0, 1);
        drawTriangle(dl, pr, v1, col(c, 0.30f), 0, 1);
        for (int i = 0; i < 4; i++) drawSegment(dl, pr, pr.toView(q[i]), pr.toView(q[(i + 1) % 4]), col(c, 0.9f), 1.5f);
        drawSegment(dl, pr, pr.toView(p.point), pr.toView(p.point + p.normal * 0.2f), col(c, 0.9f), 1.5f);
    }
}

void CollisionView::drawDrone(ImDrawList *dl, const Projection &pr) {
    Lib &L = lib();
    if (!L.droneExists) return;
    const fpv_frame &f = L.lastFrame;
    const CoordConverter &cv = L.conv;
    const DronePreset &p = currentPreset();
    Vec3 pos = cv.posToInternal({f.position[0], f.position[1], f.position[2]});
    Quat rot = cv.rotToInternal({f.rotation[0], f.rotation[1], f.rotation[2], f.rotation[3]});
    Vec3 up = rotate(rot, {0, 1, 0}), ax = rotate(rot, {1, 0, 0}), az = rotate(rot, {0, 0, 1});
    buildDroneShape(p, m_shape);
    const double simTime = f.sim_time;

    if (m_colliders) {
        for (const ShapeSphere &s : m_shape) {
            Vec3 c = pos + rotate(rot, s.local);
            Vec3 v = pr.toView(c);
            if (v.z < pr.nearZ || length(v) < s.radius) continue;  // the camera is in it (the canopy)
            float x, y, ux, uy;
            pr.screen(v, x, y);
            pr.screen(pr.toView(c + pr.up * s.radius), ux, uy);
            float r = std::max(1.5f, std::hypot(ux - x, uy - y));
            Rgb cc = s.motor < 0 ? kBody : kCollider;
            if (s.fine && r < 4) {
                dl->AddCircleFilled(ImVec2(x, y), r, col(cc, 0.8f));
            } else {
                dl->AddCircleFilled(ImVec2(x, y), r, col(cc, 0.12f));
                dl->AddCircle(ImVec2(x, y), r, col(cc, 0.9f), 0, 1.5f);
            }
        }
        // the prop discs: what breaks a prop when it touches
        for (int i = 0; i < 4; i++) {
            const Vec3 &lp = p.physics.quadMotorPos[i];
            Vec3 hub = pos + rotate(rot, lp);
            float broken = std::clamp(f.motors[i].prop_damage, 0.0f, 1.0f);
            Rgb c = mix(kProp, kKillsProps, broken);
            const int seg = 32;
            Vec3 ring[seg];
            for (int k = 0; k < seg; k++) {
                float a = 2 * kPi * k / seg;
                ring[k] = hub + (ax * std::cos(a) + az * std::sin(a)) * p.collision.propRadius;
            }
            Vec3 hv = pr.toView(hub);
            for (int k = 0; k < seg; k++) {
                Vec3 v[3] = {hv, pr.toView(ring[k]), pr.toView(ring[(k + 1) % seg])};
                drawTriangle(dl, pr, v, col(c, broken >= 1 ? 0.06f : 0.16f), 0, 1);
                drawSegment(dl, pr, v[1], v[2], col(c, 0.9f), 1.5f);
            }
        }
    }

    if (!m_contacts) return;
    // the contacts at the drawn pose: every sphere, then the prop discs
    std::lock_guard<std::mutex> lock(L.world.mutex());
    m_contactScratch.clear();
    for (const ShapeSphere &s : m_shape) L.world.sphereContacts(pos + rotate(rot, s.local), s.radius, simTime, m_contactScratch);
    size_t sphereCount = m_contactScratch.size();
    for (int i = 0; i < 4; i++) {
        Vec3 hub = pos + rotate(rot, p.physics.quadMotorPos[i]);
        size_t from = m_contactScratch.size();
        L.world.sphereContacts(hub, p.collision.propRadius, simTime, m_contactScratch);
        // only what is in the disc's plane strikes the prop
        m_contactScratch.erase(std::remove_if(m_contactScratch.begin() + (std::ptrdiff_t)from, m_contactScratch.end(),
                                              [&](const Contact &c) {
                                                  return std::fabs(dot(c.point - hub, up)) > kPropDiscHalfThickness;
                                              }),
                               m_contactScratch.end());
    }
    for (size_t i = 0; i < m_contactScratch.size(); i++) {
        const Contact &c = m_contactScratch[i];
        Material m = L.world.material(c.material);
        Rgb cc = (m.flags & kMatKillsProps) ? kKillsProps : i >= sphereCount ? kProp : kContact;
        Vec3 v = pr.toView(c.point);
        if (v.z < pr.nearZ) continue;
        float x, y;
        pr.screen(v, x, y);
        dl->AddCircleFilled(ImVec2(x, y), 3.5f, col(cc, 1));
        drawSegment(dl, pr, v, pr.toView(c.point + c.normal * 0.1f), col(cc, 1), 2.0f);
    }
}

void CollisionView::collectImpacts(double now) {
    Lib &L = lib();
    const fpv_frame &f = L.lastFrame;
    if (L.lastFrameValid && f.sim_time != m_lastSimTime) {
        m_lastSimTime = f.sim_time;
        const CoordConverter &cv = L.conv;
        for (uint32_t i = 0; i < std::min<uint32_t>(f.impact_count, FPV_MAX_IMPACTS); i++) {
            const fpv_impact &im = f.impacts[i];
            Impact e;
            e.point = cv.posToInternal({im.position[0], im.position[1], im.position[2]});
            e.normal = cv.dirToInternal({im.normal[0], im.normal[1], im.normal[2]});
            e.speed = cv.lenToInternal(im.speed);
            e.material = im.material;
            e.motor = im.motor_index;
            e.at = now;
            m_impacts.push_back(e);
        }
    }
    m_impacts.erase(std::remove_if(m_impacts.begin(), m_impacts.end(),
                                   [&](const Impact &e) { return now - e.at > kImpactShowTime || now < e.at; }),
                    m_impacts.end());
}

void CollisionView::drawImpacts(ImDrawList *dl, const Projection &pr, double now) {
    collectImpacts(now);
    Lib &L = lib();
    for (const Impact &e : m_impacts) {
        Vec3 v = pr.toView(e.point);
        if (v.z < pr.nearZ) continue;
        float age = (float)(now - e.at), fade = 1 - age / (float)kImpactShowTime;
        bool vtx = e.motor != FPV_IMPACT_CARGO && e.speed > kVtxHitSpeed;
        Rgb c = vtx ? kVtxHit : (L.world.material(e.material).flags & kMatKillsProps) ? kKillsProps
                : e.motor >= 0 ? kProp : kHit;
        float x, y;
        pr.screen(v, x, y);
        dl->AddCircle(ImVec2(x, y), 6 + 40 * age, col(c, fade), 0, 2.0f);
        drawSegment(dl, pr, v, pr.toView(e.point + e.normal * 0.25f), col(c, fade), 2.0f);
        char text[48];
        std::snprintf(text, sizeof text, "%.1f m/s%s%s", e.speed, e.motor >= 0 ? " prop" : "", vtx ? " VTX" : "");
        dl->AddText(ImVec2(x + 8, y - 8 - 20 * age), col(c, fade), text);
    }
}

void CollisionView::drawLegend(ImDrawList *dl, float w, float h) {
    if (!m_legend) return;
    double t = ImGui::GetTime();
    if (m_legendUntil < 0) m_legendUntil = t + kLegendTime;
    if (t > m_legendUntil) {
        m_legend = false;  // until a view is chosen again
        return;
    }
    Lib &L = lib();
    struct Key {
        Rgb c;
        const char *name;
    };
    std::vector<Key> keys = {{kSurface, "surface"},
                             {kObject, "moving object"},
                             {kKillsProps, "breaks the props"},
                             {kVideoClear, "clear to the video signal"},
                             {kNoCollision, "no collision, blocks video"},
                             {kPlane, "raycast callback plane"}};
    if (m_colliders) {
        keys.push_back({kBody, "drone body, canopy"});
        keys.push_back({kCollider, "motors, prop tips"});
        keys.push_back({kProp, "prop discs, prop strikes"});
    }
    if (m_contacts) {
        keys.push_back({kContact, "contact"});
        keys.push_back({kVtxHit, "hit that damages the VTX"});
    }
    bool planes;
    {
        std::lock_guard<std::mutex> lock(L.world.mutex());
        planes = !L.world.proxyPlanes().empty();
    }
    char status[160];
    if (!L.lastFrameValid)
        std::snprintf(status, sizeof status, "no drone flying");
    else if (m_noWorld && !planes)
        std::snprintf(status, sizeof status, "no collision geometry from the game");
    else if (m_noWorld)
        std::snprintf(status, sizeof status, "no triangles: the game uses the raycast callback");
    else if (m_truncated)
        std::snprintf(status, sizeof status, "%zu triangles, the nearest (of %zu+ in range)", m_drawn, m_near);
    else
        std::snprintf(status, sizeof status, "%zu triangles", m_drawn);

    const float fs = ImGui::GetFontSize(), pad = fs * 0.5f, line = fs * 1.2f;
    const char *title = "Collision view (F8 > Camera & sound)";
    float tw = std::max(ImGui::CalcTextSize(title).x, ImGui::CalcTextSize(status).x);
    for (const Key &k : keys) tw = std::max(tw, fs * 1.4f + ImGui::CalcTextSize(k.name).x);
    float bh = pad * 2 + line * (2 + keys.size());
    ImVec2 o(pad, h - bh - pad);
    if (o.y < 0) o.y = 0;
    (void)w;
    dl->AddRectFilled(o, ImVec2(o.x + tw + pad * 2, o.y + bh), IM_COL32(0, 0, 0, 170), 4.0f);
    float y = o.y + pad;
    dl->AddText(ImVec2(o.x + pad, y), IM_COL32(255, 255, 255, 255), title);
    y += line;
    dl->AddText(ImVec2(o.x + pad, y), IM_COL32(190, 190, 190, 255), status);
    y += line;
    for (const Key &k : keys) {
        dl->AddRectFilled(ImVec2(o.x + pad, y + fs * 0.2f), ImVec2(o.x + pad + fs * 0.9f, y + fs), col(k.c, 1), 2.0f);
        dl->AddText(ImVec2(o.x + pad + fs * 1.4f, y), IM_COL32(230, 230, 230, 255), k.name);
        y += line;
    }
}

}  // namespace fpv
