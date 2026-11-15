// libfpv - owner of the memory behind a fpv_draw_data.
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#pragma once

#include <vector>

#include "libfpv.h"

namespace fpv {

struct DrawListStorage {
    std::vector<fpv_draw_vertex> vertices;
    std::vector<uint32_t> indices;
    std::vector<fpv_draw_cmd> commands;
};

class DrawBuffer {
public:
    void clear() {
        m_used = 0;
        m_data = fpv_draw_data{};
    }
    DrawListStorage &newList() {
        if (m_used == m_storage.size()) m_storage.emplace_back();
        DrawListStorage &s = m_storage[m_used++];
        s.vertices.clear();
        s.indices.clear();
        s.commands.clear();
        return s;
    }
    // Finalises the pointers; valid until the next clear().
    const fpv_draw_data *finish(float width, float height) {
        m_lists.resize(m_used);
        m_data = fpv_draw_data{};
        m_data.display_size[0] = width;
        m_data.display_size[1] = height;
        for (size_t i = 0; i < m_used; i++) {
            DrawListStorage &s = m_storage[i];
            fpv_draw_list &l = m_lists[i];
            l.vertices = s.vertices.data();
            l.vertex_count = (uint32_t)s.vertices.size();
            l.indices = s.indices.data();
            l.index_count = (uint32_t)s.indices.size();
            l.commands = s.commands.data();
            l.command_count = (uint32_t)s.commands.size();
            m_data.total_vertex_count += l.vertex_count;
            m_data.total_index_count += l.index_count;
        }
        m_data.list_count = (uint32_t)m_used;
        m_data.lists = m_lists.data();
        return &m_data;
    }

private:
    std::vector<DrawListStorage> m_storage;
    std::vector<fpv_draw_list> m_lists;
    size_t m_used = 0;
    fpv_draw_data m_data{};
};

}  // namespace fpv
