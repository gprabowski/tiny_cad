#include <registry.h>

#include <constructors.h>
#include <shader_manager.h>

namespace ecs {

using r = registry;

void registry::load_from_scene(const MG1::Scene &scene) {
  auto &sm = shader_manager::get_manager();
  reset();
  std::unordered_map<uint32_t, EntityType> idx_scene_to_ecs;

  for (const auto &p : scene.points) {
    transformation t;
    t.translation = {p.position.x, p.position.y, p.position.z};
    const auto pidx = constructors::add_point(
        std::move(t), sm.programs[shader_t::POINT_SHADER].idx);
    idx_scene_to_ecs[p.GetId()] = pidx;
  }

  for (const auto &tor : scene.tori) {

    transformation t;
    t.translation = {tor.position.x, tor.position.y, tor.position.z};
    t.rotation = {tor.rotation.x, t.rotation.y, t.rotation.z};
    t.scale = {tor.scale.x, t.scale.y, t.scale.z};

    torus_params tp;
    tp.radii[1] = tor.largeRadius;
    tp.radii[0] = tor.smallRadius;

    parametric p{0.0f,
                 2 * glm::pi<float>(),
                 0.0f,
                 2 * glm::pi<float>(),
                 {tor.samples.x, tor.samples.y}};

    const auto toridx =
        constructors::add_torus(std::move(p), std::move(t), std::move(tp),
                                sm.programs[shader_t::TORUS_SHADER].idx);

    idx_scene_to_ecs[tor.GetId()] = toridx;
  }

  // bezier curve C0
  for (const auto &bc0 : scene.bezierC0) {
    std::vector<ecs::EntityType> points;
    for (const auto pidx : bc0.controlPoints) {
      points.push_back(idx_scene_to_ecs[pidx.GetId()]);
    }
    constructors::add_bezier_impl(
        sm.programs[shader_t::BEZIER_CURVE_SHADER].idx, points);
  }

  // bezier curve C2
  for (const auto &bc2 : scene.bezierC2) {
    std::vector<ecs::EntityType> points;
    for (const auto pidx : bc2.controlPoints) {
      points.push_back(idx_scene_to_ecs[pidx.GetId()]);
    }
    constructors::add_bspline_impl(
        sm.programs[shader_t::BSPLINE_CURVE_SHADER].idx, points);
  }

  // interpolation bezier curve C2
  for (const auto &ic2 : scene.interpolatedC2) {
    std::vector<ecs::EntityType> points;
    for (const auto pidx : ic2.controlPoints) {
      points.push_back(idx_scene_to_ecs[pidx.GetId()]);
    }
    constructors::add_icurve_impl(
        sm.programs[shader_t::INTERPOLATION_CURVE_SHADER].idx, points);
  }

  // surface C0
  for (const auto &sc0 : scene.surfacesC0) {
    bool ucyllinder = sc0.uWrapped;
    std::vector<ecs::EntityType> points(
        (sc0.size.x * 3 + (ucyllinder ? 0 : 1)) * (sc0.size.y * 3 + 1));

    auto samples =
        glm::vec2{sc0.patches[0].samples.x, sc0.patches[0].samples.y};
    if (sc0.size.x < 2) {
      reset();
      ImGui::OpenPopup("File Corrupted");
      return;
    }
    for (uint32_t ypat = 0; ypat < sc0.size.y; ++ypat) {
      for (uint32_t xpat = 0; xpat < sc0.size.x; ++xpat) {
        // build points
        auto &cp = sc0.patches[ypat * sc0.size.x + xpat];
        for (auto jj = 0; jj < 4; ++jj) {
          for (auto ii = 0; ii < 4; ++ii) {
            const auto patch_col_offset =
                ucyllinder ? 3 * ((3 * sc0.size.x) * ypat)
                           : 3 * ((3 * sc0.size.x + 1) * ypat);
            const auto patch_row_offset = 3 * xpat;
            const auto local_col_offset =
                ucyllinder ? (3 * sc0.size.x) * jj : (3 * sc0.size.x + 1) * jj;
            const auto local_row_offset = ii;

            const auto pl_f = (patch_col_offset + local_col_offset);
            const auto pl_s =
                ucyllinder
                    ? (patch_row_offset + local_row_offset) % (3 * sc0.size.x)
                    : (patch_row_offset + local_row_offset);
            const auto tmp_idx = pl_f + pl_s;

            points[tmp_idx] =
                idx_scene_to_ecs[cp.controlPoints[jj * 4 + ii].GetId()];
          }
        }
      }
    }

    unsigned int patches[2] = {sc0.size.x, sc0.size.y};
    constructors::add_bezier_surface(points, patches, ucyllinder, samples);
  }
  // surface C2
  for (const auto &sc2 : scene.surfacesC2) {
    bool ucyllinder = sc2.uWrapped;
    std::vector<ecs::EntityType> points(
        (4 + sc2.size.x - (ucyllinder ? 4 : 1)) * (4 + sc2.size.y - 1));
    auto samples =
        glm::vec2{sc2.patches[0].samples.x, sc2.patches[0].samples.y};
    if (sc2.size.x < 3) {
      reset();
      ImGui::OpenPopup("File Corrupted");
      return;
    }
    for (uint32_t ypat = 0; ypat < sc2.size.y; ++ypat) {
      for (uint32_t xpat = 0; xpat < sc2.size.x; ++xpat) {
        // build points
        auto &cp = sc2.patches[ypat * sc2.size.x + xpat];
        for (auto jj = 0; jj < 4; ++jj) {
          for (auto ii = 0; ii < 4; ++ii) {
            const auto patch_col_offset = ucyllinder
                                              ? ypat * (4 + sc2.size.x - 1 - 3)
                                              : ypat * (4 + sc2.size.x - 1);
            const auto patch_row_offset = xpat;
            const auto local_col_offset = ucyllinder
                                              ? (4 + sc2.size.x - 1 - 3) * (jj)
                                              : (4 + sc2.size.x - 1) * (jj);
            const auto local_row_offset = ii;

            const auto pl_f = (patch_col_offset + local_col_offset);
            const auto pl_s = ucyllinder
                                  ? (patch_row_offset + local_row_offset) %
                                        (4 + sc2.size.x - 1 - 3)
                                  : (patch_row_offset + local_row_offset);
            const auto idx = pl_f + pl_s;

            points[idx] =
                idx_scene_to_ecs[cp.controlPoints[jj * 4 + ii].GetId()];
          }
        }
      }
    }
    unsigned int patches[2] = {sc2.size.x, sc2.size.y};
    constructors::add_bspline_surface(points, patches, ucyllinder, samples);
  }
}

void registry::get_scene(MG1::Scene &scene) {
  EntityType local_counter = counter;
  std::unordered_map<uint32_t, EntityType> idx_scene_to_ecs;
  scene.Clear();

  // points
  for (const auto &[p, _] : get_map<tag_point>()) {
    if (has_component<tag_center_of_weight>(p) ||
        has_component<tag_virtual>(p)) {
      continue;
    }
    MG1::Point point;
    point.SetId(p);

    auto &f = get_component<tag_figure>(p);
    point.name = f.name;

    auto &t = get_component<transformation>(p);

    point.position = {t.translation.x, t.translation.y, t.translation.z};
    scene.points.push_back(point);
  }

  for (const auto &[tor, _] : get_map<torus_params>()) {
    MG1::Torus torus;
    torus.SetId(tor);

    auto &f = get_component<tag_figure>(tor);
    torus.name = f.name;

    auto &t = get_component<transformation>(tor);
    torus.position = {t.translation.x, t.translation.y, t.translation.z};
    torus.rotation = {t.rotation.x, t.rotation.y, t.rotation.z};
    torus.scale = {t.scale.x, t.scale.y, t.scale.z};

    auto &tp = get_component<torus_params>(tor);
    torus.largeRadius = tp.radii[1];
    torus.smallRadius = tp.radii[0];

    auto &par = get_component<parametric>(tor);
    torus.samples = {par.samples[1], par.samples[0]};

    scene.tori.push_back(torus);
  }

  // bezier curve C0
  for (const auto &[bc0, _] : get_map<bezierc>()) {
    MG1::BezierC0 bezierc0;
    bezierc0.SetId(bc0);

    auto &f = get_component<tag_figure>(bc0);
    bezierc0.name = f.name;

    auto &rel = get_component<relationship>(bc0);

    for (const auto pidx : rel.children) {
      bezierc0.controlPoints.push_back(pidx);
    }

    scene.bezierC0.push_back(bezierc0);
  }

  // bezier curve C2
  for (const auto &[bc2, _] : get_map<bspline>()) {
    MG1::BezierC2 bezierc2;
    bezierc2.SetId(bc2);

    auto &f = get_component<tag_figure>(bc2);
    bezierc2.name = f.name;

    auto &rel = get_component<relationship>(bc2);

    for (const auto pidx : rel.children) {
      bezierc2.controlPoints.push_back(pidx);
    }

    scene.bezierC2.push_back(bezierc2);
  }

  // bezier curve C2
  for (const auto &[ic, _] : get_map<icurve>()) {
    MG1::InterpolatedC2 interp;
    interp.SetId(ic);

    auto &f = get_component<tag_figure>(ic);
    interp.name = f.name;

    auto &rel = get_component<relationship>(ic);

    for (const auto pidx : rel.children) {
      interp.controlPoints.push_back(pidx);
    }

    scene.interpolatedC2.push_back(interp);
  }

  // surface C0
  for (const auto &[sc0, bsp] : get_map<bezier_surface_params>()) {
    MG1::BezierSurfaceC0 surfacec0;
    surfacec0.SetId(sc0);

    auto &f = get_component<tag_figure>(sc0);
    surfacec0.name = f.name;

    auto &rel = get_component<relationship>(sc0);
    auto &g = get_component<gl_object>(sc0);

    surfacec0.size = {bsp.u, bsp.v};

    surfacec0.uWrapped = surfacec0.vWrapped = false;
    if (bsp.cyllinder) {
      surfacec0.uWrapped = true;
    }

    for (unsigned int j = 0; j < bsp.v; ++j) {
      for (unsigned int i = 0; i < bsp.u; ++i) {
        MG1::BezierPatchC0 patchc0;
        patchc0.SetId(++local_counter);
        patchc0.name = "";
        patchc0.samples = {static_cast<uint32_t>(g.tesselation_inner.x),
                           static_cast<uint32_t>(g.tesselation_outer.x)};
        if (!bsp.cyllinder) {
          for (auto jj = 0; jj < 4; ++jj) {
            for (auto ii = 0; ii < 4; ++ii) {
              auto tmp_idx =
                  (3 * ((3 * bsp.u + 1) * j + i) + (3 * bsp.u + 1) * jj + ii);
              patchc0.controlPoints.push_back(rel.children[tmp_idx]);
            }
          }
        } else {
          for (auto jj = 0; jj < 4; ++jj) {
            for (auto ii = 0; ii < 4; ++ii) {
              const auto patch_col_offset = 3 * ((3 * bsp.u) * j);
              const auto patch_row_offset = 3 * i;
              const auto local_col_offset = (3 * bsp.u) * jj;
              const auto local_row_offset = ii;
              auto tmp_idx =
                  (patch_col_offset + local_col_offset +
                   (patch_row_offset + local_row_offset) % (bsp.u * 3));
              patchc0.controlPoints.push_back(rel.children[tmp_idx]);
            }
          }
        }
        surfacec0.patches.push_back(patchc0);
      }
    }

    scene.surfacesC0.push_back(surfacec0);
  }

  // surface C2
  for (const auto &[sc2, bsp] : get_map<bspline_surface_params>()) {
    MG1::BezierSurfaceC2 surfacec2;
    surfacec2.SetId(sc2);

    auto &f = get_component<tag_figure>(sc2);
    surfacec2.name = f.name;

    auto &rel = get_component<relationship>(sc2);
    auto &g = get_component<gl_object>(sc2);

    surfacec2.size = {bsp.u, bsp.v};

    surfacec2.uWrapped = surfacec2.vWrapped = false;
    if (bsp.cyllinder) {
      surfacec2.uWrapped = true;
    }

    for (unsigned int j = 0; j < bsp.v; ++j) {
      for (unsigned int i = 0; i < bsp.u; ++i) {
        MG1::BezierPatchC2 patchc2;
        patchc2.SetId(++local_counter);
        patchc2.samples = {4, 4};
        patchc2.samples = {static_cast<uint32_t>(g.tesselation_inner.x),
                           static_cast<uint32_t>(g.tesselation_outer.x)};
        if (!bsp.cyllinder) {
          for (auto jj = 0; jj < 4; ++jj) {
            for (auto ii = 0; ii < 4; ++ii) {
              const auto patch_col_offset = j * (4 + bsp.u - 1);
              const auto patch_row_offset = i;
              const auto local_col_offset = (4 + bsp.u - 1) * (jj);
              const auto local_row_offset = ii;
              const auto idx = (patch_col_offset + local_col_offset +
                                patch_row_offset + local_row_offset);
              patchc2.controlPoints.push_back(rel.children[idx]);
            }
          }
        } else {
          for (auto jj = 0; jj < 4; ++jj) {
            for (auto ii = 0; ii < 4; ++ii) {
              const auto patch_col_offset = j * (4 + bsp.u - 1 - 3);
              const auto patch_row_offset = i;
              const auto local_col_offset = (4 + bsp.u - 1 - 3) * (jj);
              const auto local_row_offset = ii;
              const auto idx =
                  (patch_col_offset + local_col_offset +
                   (patch_row_offset + local_row_offset) % (4 + bsp.u - 1 - 3));
              patchc2.controlPoints.push_back(rel.children[idx]);
            }
          }
        }
        surfacec2.patches.push_back(patchc2);
      }
    }
    scene.surfacesC2.push_back(surfacec2);
  }
}

void registry::reset() {
  // 1. remove all components
  reset_all_components();
  // 2. reset entities
  entities.clear();
  // 3. reset counter
  counter = 0;
  // 4. add cursor
  constructors::setup_initial_geometry();
}

ecs::EntityType r::add_entity() {
  const auto ret = counter++;
  entities.insert(std::make_pair(ret, 0));
  return ret;
}

void r::delete_entity(ecs::EntityType idx) {
  if (!exists(idx)) {
    return;
  }

  if (has_component<relationship>(idx)) {
    auto &r = get_component<relationship>(idx);
    if (r.indestructible_counter > 0) {
      return;
    }
  }

  remove_all_components(idx);
  entities.erase(idx);
}

} // namespace ecs
