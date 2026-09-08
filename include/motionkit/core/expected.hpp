// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <type_traits>

namespace motionkit {

/// An enum usable as the failure channel of Expected.
///
/// The `None` enumerator is required rather than conventional: Expected
/// default-constructs its error member, so a type without a name for success
/// would make "no error" unrepresentable and every default-constructed
/// Expected would claim to hold a value it does not have.
template <typename E>
concept ErrorEnum = std::is_enum_v<E> && requires { E::None; };

/// A value, or the reason there isn't one.
///
/// Deliberately not std::expected. Every failure in this library is an
/// ordinary, predictable outcome on a path a control loop runs every cycle --
/// a frame that is not calibrated yet, a move whose limits do not admit a
/// profile -- and the caller is expected to branch on it, not to unwind.
/// A plain aggregate keeps the object trivially copyable and the branch
/// visible at the call site.
///
/// The error type is spelled out at every use rather than defaulted, because
/// the set of things that can go wrong is part of a function's signature and
/// worth reading.
///
/// `value` is meaningful only when the error is None. It is default-constructed
/// otherwise -- never a partially filled result, which is the failure mode this
/// type exists to prevent.
template <typename T, ErrorEnum E>
struct Expected {
  /// The result, meaningful only when `error` is `None`. Default-constructed
  /// otherwise, so reading it after a failure yields a zeroed T rather than
  /// garbage -- still a bug, but a deterministic one.
  T value{};
  /// Why there is no value, or `None` when there is one.
  E error{E::None};

  /// True when a value is present. Explicit, so an Expected cannot silently
  /// convert to bool in arithmetic or be compared against an integer.
  constexpr explicit operator bool() const noexcept { return error == E::None; }
  /// True when a value is present. The named form, for use where an explicit
  /// conversion would not fire -- inside a ternary, or an && chain.
  [[nodiscard]] constexpr bool hasValue() const noexcept { return error == E::None; }
};

}  // namespace motionkit
