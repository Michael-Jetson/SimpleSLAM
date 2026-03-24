#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace SimpleSLAM {

class Node {
public:
    enum class Type { Null, Bool, Int, Double, String, Array, Object };
    using Array = std::vector<Node>;
    using Object = std::unordered_map<std::string, Node>;

    Node() = default;
    Node(bool v)
        : type_(Type::Bool), bool_val_(v) {}
    Node(int v)
        : type_(Type::Int), int_val_(v) {}
    Node(double v)
        : type_(Type::Double), double_val_(v) {}
    Node(const char* v)
        : type_(Type::String), string_val_(v == nullptr ? "" : v) {}
    Node(std::string v)
        : type_(Type::String), string_val_(std::move(v)) {}
    Node(Array v)
        : type_(Type::Array), array_val_(std::move(v)) {}
    Node(Object v)
        : type_(Type::Object), object_val_(std::make_shared<Object>(std::move(v))) {}

    Type type() const { return type_; }
    bool isNull() const { return type_ == Type::Null; }
    bool isBool() const { return type_ == Type::Bool; }
    bool isInt() const { return type_ == Type::Int; }
    bool isDouble() const { return type_ == Type::Double; }
    bool isString() const { return type_ == Type::String; }
    bool isArray() const { return type_ == Type::Array; }
    bool isObject() const { return type_ == Type::Object; }

    bool asBool() const;
    int asInt() const;
    double asDouble() const;
    const std::string& asString() const;
    const Array& asArray() const;
    const Object& asObject() const;

private:
    Type type_ = Type::Null;
    bool bool_val_ = false;
    int int_val_ = 0;
    double double_val_ = 0.0;
    std::string string_val_;
    Array array_val_;
    std::shared_ptr<Object> object_val_;
};

} // namespace SimpleSLAM
