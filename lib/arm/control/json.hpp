// lib/arm/control/json.hpp — 极简 JSON（0 依赖，自包含）
// 支持 null/bool/number/string/array/object 的完整解析与序列化，
// 字符串转义（含 \uXXXX）、数字全格式（int/float/exp）。对象保序。
#pragma once
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace arm {
namespace json {

class Value;
using Array = std::vector<Value>;
using Object = std::vector<std::pair<std::string, Value>>;

class Value {
 public:
  enum class Type { Null, Bool, Number, String, Array, Object };

  Value() : type_(Type::Null) {}
  Value(std::nullptr_t) : type_(Type::Null) {}
  Value(bool b) : type_(Type::Bool), b_(b) {}
  Value(double n) : type_(Type::Number), n_(n) {}
  Value(int n) : type_(Type::Number), n_(double(n)) {}
  Value(const char* s) : type_(Type::String), s_(s) {}
  Value(std::string s) : type_(Type::String), s_(std::move(s)) {}

  static Value array() { Value v; v.type_ = Type::Array; return v; }
  static Value object() { Value v; v.type_ = Type::Object; return v; }

  Type type() const { return type_; }
  bool isNull() const { return type_ == Type::Null; }
  bool isBool() const { return type_ == Type::Bool; }
  bool isNumber() const { return type_ == Type::Number; }
  bool isString() const { return type_ == Type::String; }
  bool isArray() const { return type_ == Type::Array; }
  bool isObject() const { return type_ == Type::Object; }

  bool asBool(bool def = false) const { return isBool() ? b_ : def; }
  double asNumber(double def = 0.0) const { return isNumber() ? n_ : def; }
  const std::string& asString() const { return s_; }
  std::string asString(const std::string& def) const { return isString() ? s_ : def; }

  size_t size() const {
    if (isArray()) return arr_.size();
    if (isObject()) return obj_.size();
    return 0;
  }
  // 数组取值：越界返回共享 Null（const 引用安全），禁止 UB
  const Value& operator[](size_t i) const {
    static const Value nullv;
    return i < arr_.size() ? arr_[i] : nullv;
  }
  Value& pushBack(Value v) { arr_.push_back(std::move(v)); return *this; }

  void set(const std::string& k, Value v) {
    for (auto& p : obj_) if (p.first == k) { p.second = std::move(v); return; }
    obj_.emplace_back(k, std::move(v));
  }
  bool has(const std::string& k) const {
    for (auto& p : obj_) if (p.first == k) return true;
    return false;
  }
  const Value& get(const std::string& k) const {
    static const Value nullv;   // const：杜绝经 const_cast 误写的共享可变状态
    for (auto& p : obj_) if (p.first == k) return p.second;
    return nullv;
  }
  // 数组取值便捷（越界/类型错返回默认）
  double numAt(size_t i, double def = 0.0) const {
    return (isArray() && i < arr_.size()) ? arr_[i].asNumber(def) : def;
  }

  // ---------- 序列化 ----------
  std::string dump() const {
    std::string out;
    write(out);
    return out;
  }

  // ---------- 解析 ----------
  static bool parse(const std::string& text, Value& out) {
    size_t pos = 0;
    skipWS(text, pos);
    if (!parseValue(text, pos, out)) return false;
    skipWS(text, pos);
    return pos == text.size();
  }

 private:
  Type type_;
  bool b_ = false;
  double n_ = 0;
  std::string s_;
  Array arr_;
  Object obj_;

  static void skipWS(const std::string& t, size_t& p) {
    while (p < t.size() && (t[p] == ' ' || t[p] == '\t' || t[p] == '\n' || t[p] == '\r')) p++;
  }
  static bool lit(const std::string& t, size_t& p, const char* w) {
    size_t n = 0;
    while (w[n]) n++;
    if (t.compare(p, n, w) == 0) { p += n; return true; }
    return false;
  }
  static void writeEscaped(std::string& o, const std::string& s) {
    o += '"';
    for (unsigned char c : s) {
      switch (c) {
        case '"': o += "\\\""; break;
        case '\\': o += "\\\\"; break;
        case '\n': o += "\\n"; break;
        case '\r': o += "\\r"; break;
        case '\t': o += "\\t"; break;
        case '\b': o += "\\b"; break;
        case '\f': o += "\\f"; break;
        default:
          if (c < 0x20) { char b[8]; std::snprintf(b, sizeof b, "\\u%04x", c); o += b; }
          else o += char(c);
      }
    }
    o += '"';
  }
  void write(std::string& o) const {
    switch (type_) {
      case Type::Null: o += "null"; break;
      case Type::Bool: o += b_ ? "true" : "false"; break;
      case Type::Number: {
        if (std::isfinite(n_)) {
          char b[40];
          if (n_ == std::floor(n_) && std::abs(n_) < 1e15)
            std::snprintf(b, sizeof b, "%.0f", n_);
          else
            std::snprintf(b, sizeof b, "%.10g", n_);
          o += b;
        } else {
          o += "0";
        }
        break;
      }
      case Type::String: writeEscaped(o, s_); break;
      case Type::Array: {
        o += '[';
        for (size_t i = 0; i < arr_.size(); i++) {
          if (i) o += ',';
          arr_[i].write(o);
        }
        o += ']';
        break;
      }
      case Type::Object: {
        o += '{';
        for (size_t i = 0; i < obj_.size(); i++) {
          if (i) o += ',';
          writeEscaped(o, obj_[i].first);
          o += ':';
          obj_[i].second.write(o);
        }
        o += '}';
        break;
      }
    }
  }

  static bool parseString(const std::string& t, size_t& p, std::string& out) {
    if (p >= t.size() || t[p] != '"') return false;
    p++;
    out.clear();
    while (p < t.size()) {
      char c = t[p];
      if (c == '"') { p++; return true; }
      if (c == '\\') {
        p++;
        if (p >= t.size()) return false;
        char e = t[p++];
        switch (e) {
          case '"': out += '"'; break;
          case '\\': out += '\\'; break;
          case '/': out += '/'; break;
          case 'b': out += '\b'; break;
          case 'f': out += '\f'; break;
          case 'n': out += '\n'; break;
          case 'r': out += '\r'; break;
          case 't': out += '\t'; break;
          case 'u': {
            if (p + 4 > t.size()) return false;
            unsigned cp = 0;
            for (int i = 0; i < 4; i++) {
              char h = t[p++];
              cp <<= 4;
              if (h >= '0' && h <= '9') cp |= unsigned(h - '0');
              else if (h >= 'a' && h <= 'f') cp |= unsigned(h - 'a' + 10);
              else if (h >= 'A' && h <= 'F') cp |= unsigned(h - 'A' + 10);
              else return false;
            }
            // UTF-8 编码（BMP；代理对按码点直拼）
            if (cp < 0x80) out += char(cp);
            else if (cp < 0x800) {
              out += char(0xC0 | (cp >> 6));
              out += char(0x80 | (cp & 0x3F));
            } else if (cp < 0x10000) {
              out += char(0xE0 | (cp >> 12));
              out += char(0x80 | ((cp >> 6) & 0x3F));
              out += char(0x80 | (cp & 0x3F));
            } else {
              out += char(0xF0 | (cp >> 18));
              out += char(0x80 | ((cp >> 12) & 0x3F));
              out += char(0x80 | ((cp >> 6) & 0x3F));
              out += char(0x80 | (cp & 0x3F));
            }
            break;
          }
          default: return false;
        }
      } else {
        out += c;
        p++;
      }
    }
    return false;
  }

  static constexpr int kMaxParseDepth = 200;   // 嵌套深度上限（真·深度限制；防深层嵌套爆栈）
  static bool parseValue(const std::string& t, size_t& p, Value& out, int depth = 0) {
    if (depth > kMaxParseDepth) return false;
    skipWS(t, p);
    if (p >= t.size()) return false;
    char c = t[p];
    if (c == 'n') { if (!lit(t, p, "null")) return false; out = Value(nullptr); return true; }
    if (c == 't') { if (!lit(t, p, "true")) return false; out = Value(true); return true; }
    if (c == 'f') { if (!lit(t, p, "false")) return false; out = Value(false); return true; }
    if (c == '"') { std::string s; if (!parseString(t, p, s)) return false; out = Value(std::move(s)); return true; }
    if (c == '[') {
      p++;
      out = Value::array();
      skipWS(t, p);
      if (p < t.size() && t[p] == ']') { p++; return true; }
      while (true) {
        Value e;
        if (!parseValue(t, p, e, depth + 1)) return false;
        out.pushBack(std::move(e));
        skipWS(t, p);
        if (p < t.size() && t[p] == ',') { p++; continue; }
        if (p < t.size() && t[p] == ']') { p++; return true; }
        return false;
      }
    }
    if (c == '{') {
      p++;
      out = Value::object();
      skipWS(t, p);
      if (p < t.size() && t[p] == '}') { p++; return true; }
      while (true) {
        skipWS(t, p);
        std::string k;
        if (!parseString(t, p, k)) return false;
        skipWS(t, p);
        if (p >= t.size() || t[p] != ':') return false;
        p++;
        Value v;
        if (!parseValue(t, p, v, depth + 1)) return false;
        out.set(k, std::move(v));
        skipWS(t, p);
        if (p < t.size() && t[p] == ',') { p++; continue; }
        if (p < t.size() && t[p] == '}') { p++; return true; }
        return false;
      }
    }
    // number：严格 JSON 语法  -?(0|[1-9][0-9]*)(''.''[0-9]+)?([eE][+-]?[0-9]+)?
    // （拒绝 1..2 / --5 / 1e5e5 / +5 / 01 / 尾部垃圾——strtod 的宽松吸收不再放行）
    size_t start = p;
    if (p < t.size() && t[p] == '-') p++;
    if (p >= t.size() || !std::isdigit((unsigned char)t[p])) return false;
    if (t[p] == '0') {
      p++;                                        // 0 之后不得再接数字（拒绝 01）
      if (p < t.size() && std::isdigit((unsigned char)t[p])) return false;
    } else {
      while (p < t.size() && std::isdigit((unsigned char)t[p])) p++;
    }
    if (p < t.size() && t[p] == '.') {
      p++;
      if (p >= t.size() || !std::isdigit((unsigned char)t[p])) return false;
      while (p < t.size() && std::isdigit((unsigned char)t[p])) p++;
    }
    if (p < t.size() && (t[p] == 'e' || t[p] == 'E')) {
      p++;
      if (p < t.size() && (t[p] == '+' || t[p] == '-')) p++;
      if (p >= t.size() || !std::isdigit((unsigned char)t[p])) return false;
      while (p < t.size() && std::isdigit((unsigned char)t[p])) p++;
    }
    out = Value(std::strtod(t.substr(start, p - start).c_str(), nullptr));
    return true;
  }
};

}  // namespace json
}  // namespace arm
