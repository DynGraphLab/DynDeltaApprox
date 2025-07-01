#pragma once
#include <any>
#include <cmath>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

#include "absl/strings/str_replace.h"
#include "app/app_io.pb.h"
#include "tools/plot/gaussian_error.h"

namespace dyn_delta_approx::tools::plot {
namespace {
using dyn_delta_approx::app::app_io::FailedExperiment;
std::string escape_tex(std::string input) {
  return absl::StrReplaceAll(input, {{"&", "\\&"}, {"_", {"\\_"}}});
}
}  // namespace
class Any {
  struct Value {
    double _double = 0;
    std::string _string;
    bool b = false;
    int64_t i = 0;
    GaussianError<double> gaussian;
    app::app_io::FailReason reason = app::app_io::FailReason::Unkown;
    Value() {}
    Value(double d) : _double(d) {}
    Value(std::string s) : _string(s) {}
    Value(bool s) : b(s) {}
    Value(int64_t _i) : i(_i) {}
    Value(GaussianError<double> var) : gaussian(var) {}
    Value(app::app_io::FailReason reas) : reason(reas) {}
    auto operator<=>(const Value& b) const = default;
    bool operator==(const Value& b) const = default;

    auto operator+(Value b) {
      b._double += _double;
      b._string = _string + b._string;
      b.b += this->b;
      b.i += i;
      b.gaussian = gaussian + b.gaussian;
      return b;
    }
    auto operator/(Value b) {
      b._double = _double / b._double;
      if (b.i != 0) {
        b.i = i / b.i;
      }
      if (b.gaussian.getValue() == 0) {
        if (b._double != 0.0) {
          b.gaussian = gaussian / b._double;
        }
      } else {
        b.gaussian = gaussian / b.gaussian;
      }
      return b;
    }
  };
  Value value;
  enum class Type { MATCH_ANY, DOUBLE, STRING, BOOL, INT, GAUSS, FAIL };
  Type t = Type::MATCH_ANY;

 public:
  Any() {}
  Any(double d) : value(d), t(Type::DOUBLE) {}
  Any(std::string s) : value(s), t(Type::STRING) {}
  Any(bool s) : value(s), t(Type::BOOL) {}
  Any(int64_t _i) : value(_i), t(Type::INT) {}
  Any(GaussianError<double> var) : value(var), t(Type::GAUSS) {}
  Any(app::app_io::FailReason reas) : value(reas), t(Type::FAIL) {}
  bool isError() const { return t == Type::FAIL; }
  auto operator<=>(const Any& b) const {
    if (b.t == Type::MATCH_ANY || this->t == Type::MATCH_ANY) {
      return std::partial_ordering::equivalent;
    }
    if (b.t == Type::DOUBLE || this->t == Type::DOUBLE) {
      return to_double() <=> b.to_double();
    }
    return value <=> b.value;
  }
  bool operator==(const Any& b) const {
    if (b.t == Type::MATCH_ANY || this->t == Type::MATCH_ANY) {
      return true;
    }
    if (b.t == Type::DOUBLE || this->t == Type::DOUBLE) {
      return to_double() == b.to_double();
    }
    return value == b.value;
  }
  std::string to_string() const {
    if (t == Type::DOUBLE) {
      std::stringstream result;
      if (value._double == std::round(value._double)) {
        result << "\\numprint{" << std::fixed << std::setprecision(0) << value._double << +"}";
        return result.str();
      }
      result << std::fixed << std::setprecision(2) << value._double;

      return result.str();
    }
    if (t == Type::STRING) {
      return value._string;
    }
    if (t == Type::BOOL) {
      return std::to_string(value.b);
    }
    if (t == Type::INT) {
      return "\\numprint{" + std::to_string(value.i) + "}";
    }
    if (t == Type::GAUSS) {
      std::stringstream r;
      r << std::fixed << std::setprecision(2) << value.gaussian.getValue();
      return r.str();
    }
    if (t == Type::FAIL) {
      if (value.reason == app::app_io::FailReason::Timeout) {
        return "OOT";
      }
      if (value.reason == app::app_io::FailReason::Memory) {
        return "OOM";
      }
      return "UNK";
    }
    return "bad_cast";
  }
  double to_double() const {
    if (t == Type::DOUBLE) {
      return value._double;
    }
    if (t == Type::BOOL) {
      return value.b;
    }
    if (t == Type::INT) {
      return value.i;
    }
    if (t == Type::GAUSS) {
      return value.gaussian.getValue();
    }
    return std::numeric_limits<double>::quiet_NaN();
  }
  Any operator+(Any other) {
    other.value = other.value + value;
    return other;
  }
  Any operator/(Any other) {
    other.value = value / other.value;
    return other;
  }
  Any operator/(double d) {
    Any other = this->to_double();
    other.value = other.value / d;
    return other;
  }
};
template <class T>
class Table {
  size_t _cols, _head;
  std::vector<std::vector<std::optional<T>>> table;

 public:
  Table(size_t cols, size_t head) : _cols(cols), _head(head) {}
  void insert(std::vector<std::optional<T>> values) {
    if (values.size() != _cols) {
      throw std::runtime_error("size mismatch expected " + std::to_string(_cols) + " got " +
                               std::to_string(values.size()));
    }
    table.push_back(values);
  }
  std::string to_latex(std::string caption, std::string label, std::string colspec) {
    std::stringstream result;
    // table
    result << "\\begin{longtblr}[" << std::endl
           << "caption = {" << caption << "}," << std::endl
           << "  label = {" << label << "}," << std::endl
           << " ]{" << std::endl
           << "colspec = {" << colspec << "}," << "rowhead =" << _head
           << ","
              " hlines,  row{even} = {gray9},"
              " row{1} = {olive9},}"
           << std::endl;

    for (auto& row : table) {
      bool first = true;
      for (auto c : row) {
        if (!first) {
          result << "&";
        }
        first = false;
        if (c.has_value()) {
          result << c.value().to_latex();
        }
      }
      result << "\\\\" << std::endl;
    }
    result << "\\end{longtblr}" << std::endl;
    return result.str();
  }
};
struct Cell {
  std::string t;
  std::set<std::string> decs;

  Cell(std::string text, std::set<std::string> decorators = {}) : t(text), decs(decorators) {}
  std::string to_latex() {
    // TODO implement decorators
    std::string dec = "";
    if (decs.find("min") != decs.end() || decs.find("max") != decs.end()) {
      dec = "\\bfseries ";
    }
    if (decs.find("error") != decs.end()) {
      dec = "\\em ";
    }
    return dec + escape_tex(t);
  }
};
template <class T>
class PivotTable {
  std::set<std::string> _index;
  std::set<std::string> _pivot;
  using R = std::map<std::map<std::string, Any>, Any>;

  std::set<std::map<std::string, Any>> header;
  std::map<std::map<std::string, Any>, R> rows;

 public:
  using Row = R;

  PivotTable(
      const std::map<std::array<std::map<std::string, Any>, 2>, T>& data,
      const std::map<std::array<std::map<std::string, Any>, 2>, app::app_io::FailReason>& failed,
      std::set<std::string> index, std::set<std::string> pivot,
      std::set<std::string> virtual_headers = {},
      std::function<Any(const std::map<std::string, Any>&, const std::string&)> create_virtual =
          [](const std::map<std::string, Any>&, const std::string&) -> Any { return {}; })
      : _index(index), _pivot(pivot) {
    for (auto& [k, d] : data) {
      header.insert(k[1]);
      rows[k[0]][k[1]] = d;
    }
    for (auto h : header) {
      for (auto [k, v] : h) std::cout << k << "= " << v.to_string() << std::endl;
    }
    for (const auto& v : virtual_headers) {
      std::map<std::string, Any> entry;
      entry["runConfig_shortName"] = v;
      header.insert(entry);
      for (auto& [row, entries] : rows) {
        for (auto [k, v] : row) {
          std::cout << k << " r: " << v.to_string() << std::endl;
        }
        entries[entry] = create_virtual(row, v);
      }
    }
    for (auto& [k, d] : failed) {
      header.insert(k[1]);
      std::cout << k[0].begin()->second.to_string() << " " << k[1].begin()->second.to_string()
                << " " << Any(d).to_string() << std::endl;
      rows[k[0]][k[1]] = d;
    }
  }
  std::string colspec() {
    std::stringstream r;

    for (auto i : _index) {
      r << "l";
    }
    for (auto h : header) {
      r << "r";
    }
    return r.str();
  }
  Table<Cell> to_table(
      std::string sorted_by, bool normalize_by_max, bool pivotise,
      const app::app_io::Visualisation& vis,
      std::function<std::vector<std::set<std::string>>(Row)> row_decorator = [](Row row) {
        std::vector<std::set<std::string>> r(row.size());
        return r;
      }) {
    auto bool_params = vis.bool_params();
    std::function<bool(Any, Any)> compare = [](Any a, Any b) { return a < b; };
    if (!bool_params["minimization_problem"]) {
      compare = [](Any a, Any b) { return a > b; };
    }
    Table<Cell> result(_index.size() + header.size(), _pivot.size());
    // construct header
    for (auto p : _pivot) {
      std::vector<std::optional<Cell>> values(_index.size() - 1, std::nullopt);
      values.push_back(Cell(p));
      for (auto h : header) {
        std::stringstream col;
        for (auto [prop, value] : h) {
          if (p == prop || prop == "entry") {
            values.push_back(Cell(value.to_string(), {"header"}));
          }
        }
      }
      result.insert(values);
      if (!pivotise) {
        break;
      }
    }
    std::vector<std::pair<std::map<std::string, Any>, Row>> rows_sorted;
    for (auto [k, v] : rows) {
      if (normalize_by_max) {
        bool first = true;
        Any val = 0.0;
        for (auto& [header, value] : v) {
          if (first) {
            val = value;
          }
          first = false;
          if (compare(val, value)) {
            val = value;
          }
        }
        for (auto& [header, value] : v) {
          value = value.to_double() / val.to_double();
        }
      }
      rows_sorted.push_back({k, v});
    }
    std::stable_sort(rows_sorted.begin(), rows_sorted.end(), [&](auto a, auto b) {
      return a.first[*_index.begin()] < b.first[*_index.begin()];
    });
    if (sorted_by != "") {
      std::stable_sort(rows_sorted.begin(), rows_sorted.end(),
                       [&](auto a, auto b) { return a.first[sorted_by] < b.first[sorted_by]; });
    }
    for (auto& [entry, row] : rows_sorted) {
      std::vector<std::optional<Cell>> values;
      for (auto i : _index) {
        values.push_back(Cell(entry[i].to_string()));
      }

      auto decoratos = row_decorator(row);
      size_t i = 0;
      for (auto h : header) {
        if (row.find(h) != row.end()) {
          values.push_back(Cell(row[h].to_string(), decoratos[i++]));
        } else {
          values.push_back(std::nullopt);
        }
      }
      result.insert(values);
    }
    return result;
  }
};
}  // namespace dyn_delta_approx::tools::plot