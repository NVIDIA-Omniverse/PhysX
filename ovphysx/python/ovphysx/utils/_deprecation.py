# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-UTILS-001
# @covers AC-15

"""Factories for the deprecated ``camelCase`` spellings AC-15 bounds.

Private and never re-exported: the aliases it builds are module attributes of
the submodule that owns the new spelling, and ``ovphysx.utils`` resolves them
through its own alias table rather than through a submodule ``__all__``.

Names and keyword parameters are two mechanisms with one policy. A deprecated
name is resolved by ``ovphysx.utils.__getattr__`` and so needs a table there;
a deprecated parameter is resolved inside the call and so is recorded on the
wrapper itself, as ``__deprecated_parameters__``, which is the enumeration
point for the bound AC-15 sets.
"""

import functools
import inspect
import warnings


def _external_caller_stacklevel():
    """Return the warning stack level of the first caller outside this module."""
    frame = inspect.currentframe()
    if frame is None:
        return 2
    frame = frame.f_back
    stacklevel = 1
    while frame is not None and frame.f_globals.get("__name__") == __name__:
        stacklevel += 1
        frame = frame.f_back
    return stacklevel


def deprecated_alias(target, old_name):
    """Build a deprecated alias forwarding to ``target``.

    The alias keeps ``target``'s signature and docstring, so ``help()`` and
    ``inspect.signature`` answer for it, and warns once per call naming both
    spellings so a consumer can grep for either.

    Args:
        target:   The renamed helper the alias forwards to.
        old_name: The ``omni.physx.scripts`` spelling being kept alive.
    """
    new_name = target.__name__

    @functools.wraps(target)
    def alias(*args, **kwargs):
        warnings.warn(
            f"ovphysx.utils.{old_name} is a deprecated alias for "
            f"ovphysx.utils.{new_name}; call {new_name} instead.",
            DeprecationWarning,
            stacklevel=_external_caller_stacklevel(),
        )
        return target(*args, **kwargs)

    return alias


def deprecated_parameter(old_name, new_name):
    """Build a decorator keeping a renamed keyword parameter's old spelling alive.

    The renamed spelling is the parameter; the old one is accepted, warned about
    on the same terms as a deprecated name, and forwarded to it. Supplying both
    raises ``TypeError``, because nothing in such a call says which of the two
    values the caller meant.

    Args:
        old_name: The ``omni.physx.scripts`` spelling being kept alive.
        new_name: The renamed parameter it forwards to.
    """

    def decorate(target):
        position = list(inspect.signature(target).parameters).index(new_name)

        @functools.wraps(target)
        def wrapper(*args, **kwargs):
            if old_name in kwargs:
                if new_name in kwargs or len(args) > position:
                    raise TypeError(
                        f"ovphysx.utils.{target.__name__}() got both {new_name} and "
                        f"its deprecated spelling {old_name}; pass {new_name} alone."
                    )
                warnings.warn(
                    f"The {old_name} parameter of ovphysx.utils.{target.__name__} is "
                    f"a deprecated spelling of {new_name}; pass {new_name} instead.",
                    DeprecationWarning,
                    stacklevel=_external_caller_stacklevel(),
                )
                kwargs[new_name] = kwargs.pop(old_name)
            return target(*args, **kwargs)

        wrapper.__deprecated_parameters__ = (
            *getattr(target, "__deprecated_parameters__", ()),
            (old_name, new_name),
        )
        return wrapper

    return decorate
