// libfpv - the overlay's collision view (F8 > Camera & sound): the world as
// libfpv collides with it, drawn instead of (or over) the game's picture
// from the view the game shows. Static surfaces and moving objects coloured
// by material (what breaks props stands out), the raycast callback's
// planes, the drone's own collision spheres and prop discs, its contacts and
// the impacts it takes (the hard ones that hurt the video transmitter
// marked). A developer's tool: nothing of it is saved.
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "fpv_math.h"
#include "world/collision.h"
#include "world/drone_shape.h"

struct ImDrawList;

namespace fpv {

class CollisionView {
public:
    enum Mode { Off = 0, Solid = 1, XRay = 2 };

    // Off, or as the LIBFPV_COLLISION_VIEW environment variable says
    // ("solid" or 1, "xray" or 2): to start with it on, and for tests.
    CollisionView();

    bool on() const { return m_mode != Off; }
    // The settings, in the Camera tab.
    void ui();
    // Draws the view into `dl` (the background list, under the overlay's
    // windows) for a w x h display; `now` is the overlay's clock (s).
    void draw(ImDrawList *dl, float w, float h, double now);

private:
    // The view the game shows, in libfpv's frame, and how it reaches the
    // screen: the picture's rectangle and aspect, and the lens.
    struct Projection {
        Vec3 pos, right, up, fwd;
        float tanX = 1, tanY = 1, nearZ = 0.05f;
        float rx = 0, ry = 0, rw = 1, rh = 1;
        float k1 = 0, k2 = 0, zoom = 1, scaleX = 1, scaleY = 1;
        bool lens = false;
        Vec3 toView(Vec3 p) const { return {dot(p - pos, right), dot(p - pos, up), dot(p - pos, fwd)}; }
        // a view-space point in front of the near plane to the screen
        void screen(Vec3 v, float &x, float &y) const;
    };
    struct Impact {
        Vec3 point, normal;
        float speed;  // m/s
        uint8_t material;
        int motor;
        double at;  // overlay clock
    };
    bool setupProjection(float w, float h, Projection &pr) const;
    void drawWorld(ImDrawList *dl, const Projection &pr);
    void drawPlanes(ImDrawList *dl, const Projection &pr);
    void drawDrone(ImDrawList *dl, const Projection &pr);
    void drawImpacts(ImDrawList *dl, const Projection &pr, double now);
    void drawLegend(ImDrawList *dl, float w, float h);
    // A world triangle, near-clipped, through the lens; fill and outline.
    // `fill`: alpha 0 for none; faded into the background with the
    // distance when fogDist > 0 (each corner by its own). `edges`: which of
    // v0-v1, v1-v2, v2-v0 (bits 0..2) get an outline.
    void drawTriangle(ImDrawList *dl, const Projection &pr, const Vec3 v[3], uint32_t fill, uint32_t edge,
                      float edgeWidth, unsigned edges = 7, float fogDist = 0);
    // A world triangle (view space) into m_items: cut into pieces no larger
    // than their distance from the camera, so sorting them back to front
    // works (a ground triangle hundreds of metres across, sorted whole,
    // could come out nearest and cover all that stands on it).
    void addPieces(const Projection &pr, uint32_t index, Vec3 a, Vec3 b, Vec3 c, unsigned edges, int depth);
    void drawSegment(ImDrawList *dl, const Projection &pr, Vec3 a, Vec3 b, uint32_t col, float width);
    void collectImpacts(double now);

    int m_mode = Off;
    bool m_colliders = true, m_contacts = true;
    // the legend shows for kLegendTime s after a view is chosen;
    // until < 0: from the next draw
    bool m_legend = true;
    double m_legendUntil = -1;
    float m_distance = 150.0f;  // m
    int m_budget = 40000;       // triangles drawn at most, the nearest

    std::vector<CollisionWorld::DebugTriangle> m_tris;
    struct Item {
        float depth;     // the centroid of its part in front of the near plane
        uint32_t index;  // into m_tris
        Vec3 v[3];       // view space
        unsigned edges;  // the sides that are the triangle's own (outlined)
    };
    std::vector<Item> m_items;
    std::vector<uint32_t> m_cut;  // the triangles cut into pieces (an underlay under them)
    std::vector<ShapeSphere> m_shape;
    std::vector<Contact> m_contactScratch;
    std::vector<Impact> m_impacts;
    double m_lastSimTime = -1;
    // the last frame's counts, for the legend
    size_t m_near = 0, m_drawn = 0;
    bool m_truncated = false, m_noWorld = false;
};

}  // namespace fpv
