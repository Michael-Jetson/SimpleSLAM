#include <simpleslam/config/Node.h>

#include <stdexcept>

namespace SimpleSLAM {
namespace {

const char* typeName(Node::Type type) {
    switch (type) {
        case Node::Type::Null:
            return "null";
        case Node::Type::Bool:
            return "bool";
        case Node::Type::Int:
            return "int";
        case Node::Type::Double:
            return "double";
        case Node::Type::String:
            return "string";
        case Node::Type::Array:
            return "array";
        case Node::Type::Object:
            return "object";
    }
    return "unknown";
}

std::runtime_error typeError(Node::Type expected, Node::Type actual) {
    return std::runtime_error(
        std::string("Node type mismatch: expected ") + typeName(expected) +
        ", got " + typeName(actual));
}

} // namespace

bool Node::asBool() const {
    if (!isBool()) {
        throw typeError(Type::Bool, type_);
    }
    return bool_val_;
}

int Node::asInt() const {
    if (!isInt()) {
        throw typeError(Type::Int, type_);
    }
    return int_val_;
}

double Node::asDouble() const {
    if (isDouble()) {
        return double_val_;
    }
    if (isInt()) {
        return static_cast<double>(int_val_);
    }
    throw std::runtime_error(
        std::string("Node type mismatch: expected number, got ") + typeName(type_));
}

const std::string& Node::asString() const {
    if (!isString()) {
        throw typeError(Type::String, type_);
    }
    return string_val_;
}

const Node::Array& Node::asArray() const {
    if (!isArray()) {
        throw typeError(Type::Array, type_);
    }
    return array_val_;
}

const Node::Object& Node::asObject() const {
    if (!isObject()) {
        throw typeError(Type::Object, type_);
    }
    if (!object_val_) {
        throw std::runtime_error("Node object storage is not initialized");
    }
    return *object_val_;
}

} // namespace SimpleSLAM
