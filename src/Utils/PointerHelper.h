#pragma once

#include <memory>
#include <optional>

template <class T>
using sp = std::shared_ptr<T>;

template <class T>
using up = std::unique_ptr<T>;

template <class T>
using wp = std::weak_ptr<T>;

template <class T>
using optional = std::optional<T>;
