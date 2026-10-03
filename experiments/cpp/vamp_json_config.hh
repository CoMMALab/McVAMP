#pragma once

#include <nlohmann/json.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace vamp {
namespace config {

using json = nlohmann::json;

struct ConstraintConfig {
  std::string constraint_type;
  std::vector<double> lower_bound;
  std::vector<double> upper_bound;
};

struct ProblemConfig {
  std::vector<double> start;
  std::vector<double> goal;
};

struct Sphere {
  std::string id;
  double x = 0.0, y = 0.0, z = 0.0, r = 0.0;
};

struct Cuboid {
  std::string id;
  double x = 0.0, y = 0.0, z = 0.0;
  double yaw = 0.0, pitch = 0.0, roll = 0.0;
  double dx = 0.0, dy = 0.0, dz = 0.0;
};

struct EnvConfig {
  std::vector<Sphere> spheres;
  std::vector<Cuboid> cuboids;
};

struct Config {
  ConstraintConfig constraint;
  ProblemConfig problem;
  EnvConfig env;
};

// --- Parsing helpers (minimal) ---

inline std::vector<double> AsDoubleVector(const json& j, const char* ctx, std::size_t min = 0) {
  if (!j.is_array()) throw std::runtime_error(std::string("Expected array for ") + ctx);
  if (j.size() < min) throw std::runtime_error(std::string("Too few elements for ") + ctx);
  std::vector<double> out;
  out.reserve(j.size());
  for (const auto& v : j) {
    if (!v.is_number()) throw std::runtime_error(std::string("Non-numeric value in ") + ctx);
    out.push_back(v.get<double>());
  }
  return out;
}

inline void ParseConstraint(const json& j, ConstraintConfig& c) {
  if (!j.is_object()) throw std::runtime_error("constraint must be an object");
  if (auto it = j.find("constraint_type"); it != j.end()) {
    if (!it->is_string()) throw std::runtime_error("constraint.constraint_type must be a string");
    c.constraint_type = it->get<std::string>();
  }
  if (auto it = j.find("lower_bound"); it != j.end()) c.lower_bound = AsDoubleVector(*it, "constraint.lower_bound");
  if (auto it = j.find("upper_bound"); it != j.end()) c.upper_bound = AsDoubleVector(*it, "constraint.upper_bound");
}

inline void ParseProblem(const json& j, ProblemConfig& p) {
  if (!j.is_object()) throw std::runtime_error("problem must be an object");
  if (auto it = j.find("start"); it != j.end()) p.start = AsDoubleVector(*it, "problem.start");
  if (auto it = j.find("goal"); it != j.end()) p.goal = AsDoubleVector(*it, "problem.goal");
}

inline Sphere ParseSphereKV(const std::string& id, const json& v) {
  auto a = AsDoubleVector(v, ("env.spheres[" + id + "]").c_str(), 4);
  return Sphere{id, a[0], a[1], a[2], a[3]};
}

inline Cuboid ParseCuboidKV(const std::string& id, const json& v) {
  auto a = AsDoubleVector(v, ("env.cuboids[" + id + "]").c_str(), 9);
  Cuboid c;
  c.id = id;
  c.x = a[0]; c.y = a[1]; c.z = a[2];
  c.yaw = a[3]; c.pitch = a[4]; c.roll = a[5];
  c.dx = a[6]; c.dy = a[7]; c.dz = a[8];
  return c;
}

inline void ParseEnv(const json& j, EnvConfig& e) {
  if (!j.is_object()) throw std::runtime_error("env must be an object");
  if (auto it = j.find("spheres"); it != j.end()) {
    if (!it->is_object()) throw std::runtime_error("env.spheres must be an object");
    e.spheres.clear();
    e.spheres.reserve(it->size());
    for (auto it2 = it->begin(); it2 != it->end(); ++it2) {
      e.spheres.emplace_back(ParseSphereKV(it2.key(), it2.value()));
    }
  }
  if (auto it = j.find("cuboids"); it != j.end()) {
    if (!it->is_object()) throw std::runtime_error("env.cuboids must be an object");
    e.cuboids.clear();
    e.cuboids.reserve(it->size());
    for (auto it2 = it->begin(); it2 != it->end(); ++it2) {
      e.cuboids.emplace_back(ParseCuboidKV(it2.key(), it2.value()));
    }
  }
}

// --- Public API (minimal) ---

inline bool ParseConfigFromJson(std::string_view text, Config& out, std::string* err = nullptr) {
  try {
    const json root = json::parse(text.begin(), text.end(), nullptr, /*allow_exceptions=*/true, /*ignore_comments=*/false);
    if (auto it = root.find("constraint"); it != root.end()) ParseConstraint(*it, out.constraint);
    if (auto it = root.find("problem"); it != root.end()) ParseProblem(*it, out.problem);
    if (auto it = root.find("env"); it != root.end()) ParseEnv(*it, out.env);
    return true;
  } catch (const std::exception& e) {
    if (err) *err = e.what();
    return false;
  }
}

inline bool ParseConfigFromFile(const std::string& path, Config& out, std::string* err = nullptr) {
  std::ifstream ifs(path);
  if (!ifs) {
    if (err) *err = "Failed to open: " + path;
    return false;
  }
  std::ostringstream oss;
  oss << ifs.rdbuf();
  return ParseConfigFromJson(oss.str(), out, err);
}

}  // namespace config
}  // namespace vamp
