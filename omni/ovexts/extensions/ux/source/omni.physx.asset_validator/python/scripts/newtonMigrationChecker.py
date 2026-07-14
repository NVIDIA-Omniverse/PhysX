# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

__all__ = ["NewtonMigrationChecker", "NewtonMimicMigrationChecker"]

from omni.asset_validator.core import BaseRuleChecker, Suggestion, registerRule
from pxr import Usd, Tf, UsdPhysics, PhysxSchema


# Mapping of deprecated PhysX attributes to their Newton replacements.
# Each entry: (physx_schema_class, physx_attr_getter, newton_api_identifier, newton_attr_name)
_SCENE_MIGRATIONS = [
    # physxScene:timeStepsPerSecond -> newton:timeStepsPerSecond
    ("GetTimeStepsPerSecondAttr", "NewtonSceneAPI", "newton:timeStepsPerSecond"),
]

_ARTICULATION_MIGRATIONS = [
    # physxArticulation:enabledSelfCollisions -> newton:selfCollisionEnabled
    ("GetEnabledSelfCollisionsAttr", "NewtonArticulationRootAPI", "newton:selfCollisionEnabled"),
]

def _has_authored_physx_attr(prim, schema_class, getter_name):
    """Check if a PhysX schema attribute is authored on the prim."""
    api = schema_class.Get(prim.GetStage(), prim.GetPath())
    if not api:
        return False, None
    getter = getattr(api, getter_name, None)
    if getter is None:
        return False, None
    attr = getter()
    if attr and attr.HasAuthoredValue():
        return True, attr
    return False, None


def _apply_newton_api(prim, newton_api_name):
    """Apply a Newton API schema (single-apply) to a prim if not already present."""
    prim.ApplyAPI(newton_api_name)


def _migrate_attr(stage, prim, physx_attr, newton_api_name, newton_attr_name):
    """Migrate a deprecated PhysX attribute to its Newton equivalent (direct value copy)."""
    val = physx_attr.Get()
    if val is None:
        return

    _apply_newton_api(prim, newton_api_name)

    newton_attr = prim.GetAttribute(newton_attr_name)
    if newton_attr:
        newton_attr.Set(val)

    prim.RemoveProperty(physx_attr.GetName())


def _migrate_collision_attrs(stage, prim):
    """Migrate PhysX collision offsets to Newton margin/gap.

    Newton's contactGap is additive on top of contactMargin, whereas PhysX
    contactOffset is measured from the shape surface. So:
        newton:contactMargin = physxCollision:restOffset
        newton:contactGap    = physxCollision:contactOffset - physxCollision:restOffset
    """
    rest_authored, rest_attr = _has_authored_physx_attr(prim, PhysxSchema.PhysxCollisionAPI, "GetRestOffsetAttr")
    contact_authored, contact_attr = _has_authored_physx_attr(prim, PhysxSchema.PhysxCollisionAPI, "GetContactOffsetAttr")

    if not rest_authored and not contact_authored:
        return

    rest_val = rest_attr.Get() if rest_authored else 0.0
    contact_val = contact_attr.Get() if contact_authored else None

    _apply_newton_api(prim, "NewtonCollisionAPI")

    # Migrate restOffset -> contactMargin (direct)
    if rest_authored:
        margin_attr = prim.GetAttribute("newton:contactMargin")
        if margin_attr:
            margin_attr.Set(rest_val)
        prim.RemoveProperty(rest_attr.GetName())

    # Migrate contactOffset -> contactGap (subtract restOffset)
    if contact_authored and contact_val is not None:
        import math
        if not math.isinf(contact_val):
            gap = contact_val - rest_val
            gap_attr = prim.GetAttribute("newton:contactGap")
            if gap_attr:
                gap_attr.Set(gap)
        prim.RemoveProperty(contact_attr.GetName())


@registerRule("Omni:Physx")
class NewtonMigrationChecker(BaseRuleChecker):
    __doc__ = """
    Check for deprecated PhysX schema attributes that should be migrated to Newton schema equivalents.

    Deprecated mappings:
        physxScene:timeStepsPerSecond -> newton:timeStepsPerSecond (NewtonSceneAPI)
        physxCollision:contactOffset  -> newton:contactGap (NewtonCollisionAPI)
        physxCollision:restOffset     -> newton:contactMargin (NewtonCollisionAPI)
        physxArticulation:enabledSelfCollisions -> newton:selfCollisionEnabled (NewtonArticulationRootAPI)
    """

    DEPRECATED_SCENE_ATTR_CODE = "NewtonMigrationChecker:DeprecatedSceneAttr"
    DEPRECATED_COLLISION_ATTR_CODE = "NewtonMigrationChecker:DeprecatedCollisionAttr"
    DEPRECATED_ARTICULATION_ATTR_CODE = "NewtonMigrationChecker:DeprecatedArticulationAttr"

    def _fix_prim(self, stage: Usd.Stage, prim: Usd.Prim):
        """Apply all Newton migrations for a single prim."""
        # Scene migrations
        for getter_name, newton_api, newton_attr in _SCENE_MIGRATIONS:
            authored, attr = _has_authored_physx_attr(prim, PhysxSchema.PhysxSceneAPI, getter_name)
            if authored:
                _migrate_attr(stage, prim, attr, newton_api, newton_attr)

        # Articulation migrations
        for getter_name, newton_api, newton_attr in _ARTICULATION_MIGRATIONS:
            authored, attr = _has_authored_physx_attr(prim, PhysxSchema.PhysxArticulationAPI, getter_name)
            if authored:
                _migrate_attr(stage, prim, attr, newton_api, newton_attr)

        # Collision migrations (special: contactOffset → contactGap requires subtraction)
        _migrate_collision_attrs(stage, prim)

    def CheckPrim(self, prim: Usd.Prim):
        # Check scene attributes
        for getter_name, newton_api, newton_attr in _SCENE_MIGRATIONS:
            authored, attr = _has_authored_physx_attr(prim, PhysxSchema.PhysxSceneAPI, getter_name)
            if authored:
                self._AddFailedCheck(
                    message=f"Deprecated attribute '{attr.GetName()}' on '{prim.GetPath()}'. "
                            f"Migrate to '{newton_attr}' ({newton_api}).",
                    at=prim,
                    suggestion=Suggestion(
                        message=f"Migrate '{attr.GetName()}' to '{newton_attr}'",
                        callable=self._fix_prim,
                        at=[prim.GetStage().GetRootLayer()],
                    ),
                    code=self.DEPRECATED_SCENE_ATTR_CODE,
                )

        # Check articulation attributes
        for getter_name, newton_api, newton_attr in _ARTICULATION_MIGRATIONS:
            authored, attr = _has_authored_physx_attr(prim, PhysxSchema.PhysxArticulationAPI, getter_name)
            if authored:
                self._AddFailedCheck(
                    message=f"Deprecated attribute '{attr.GetName()}' on '{prim.GetPath()}'. "
                            f"Migrate to '{newton_attr}' ({newton_api}).",
                    at=prim,
                    suggestion=Suggestion(
                        message=f"Migrate '{attr.GetName()}' to '{newton_attr}'",
                        callable=self._fix_prim,
                        at=[prim.GetStage().GetRootLayer()],
                    ),
                    code=self.DEPRECATED_ARTICULATION_ATTR_CODE,
                )

        # Check collision attributes (contactOffset and restOffset)
        _col_deprecated = [
            ("GetContactOffsetAttr", "newton:contactGap"),
            ("GetRestOffsetAttr", "newton:contactMargin"),
        ]
        for getter_name, newton_attr in _col_deprecated:
            authored, attr = _has_authored_physx_attr(prim, PhysxSchema.PhysxCollisionAPI, getter_name)
            if authored:
                self._AddFailedCheck(
                    message=f"Deprecated attribute '{attr.GetName()}' on '{prim.GetPath()}'. "
                            f"Migrate to '{newton_attr}' (NewtonCollisionAPI).",
                    at=prim,
                    suggestion=Suggestion(
                        message=f"Migrate collision offsets to Newton contactMargin/contactGap",
                        callable=self._fix_prim,
                        at=[prim.GetStage().GetRootLayer()],
                    ),
                    code=self.DEPRECATED_COLLISION_ATTR_CODE,
                )


# ---------------------------------------------------------------------------
# PhysxMimicJointAPI  →  NewtonMimicAPI
# ---------------------------------------------------------------------------
#
# PhysX:  jointPos + gearing * refPos + offset = 0   ⇒  jointPos = -gearing * refPos - offset
# Newton: jointPos = coef0 + coef1 * refPos
# ⇒  newton:mimicCoef1 = -gearing
#    newton:mimicCoef0 = -offset
#    newton:mimicJoint = physxMimicJoint:<instance>:referenceJoint
#
# NewtonMimicAPI is single-apply per joint and supports single-DOF followers and
# leaders only (PhysicsRevoluteJoint / PhysicsPrismaticJoint). Joints that don't
# meet those criteria are left alone by this checker.


_PHYSX_MIMIC_API_PREFIX = "PhysxMimicJointAPI:"


def _collect_physx_mimic_instances(prim):
    """Return the list of PhysxMimicJointAPI instance names applied to prim."""
    return [s.split(":", 1)[1] for s in prim.GetAppliedSchemas()
            if s.startswith(_PHYSX_MIMIC_API_PREFIX)]


def _is_single_dof_joint(prim):
    return prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint)


def _can_migrate_mimic(prim):
    """True if the migration from PhysxMimicJointAPI to NewtonMimicAPI is well-defined:
    exactly one PhysxMimicJointAPI instance is applied, the follower is a single-DOF
    joint, and any resolvable leader target is also single-DOF.

    Other scene-validity concerns (dangling referenceJoint target, articulation
    membership, revolute limit authoring, ...) are deliberately not checked here —
    they are enforced by the NewtonMimicAPI runtime and are not specific to this
    migration.
    """
    instances = _collect_physx_mimic_instances(prim)
    if len(instances) != 1:
        return False, None
    if not _is_single_dof_joint(prim):
        return False, None

    api = PhysxSchema.PhysxMimicJointAPI.Get(prim, instances[0])
    if not api:
        return False, None

    # NewtonMimicAPI's mimicJoint relationship has no per-axis selector, so a
    # multi-DOF leader cannot be migrated faithfully. Unresolvable targets are
    # passed through for the runtime to report.
    for target in api.GetReferenceJointRel().GetTargets():
        leader = prim.GetStage().GetPrimAtPath(target)
        if leader and leader.IsValid() and not _is_single_dof_joint(leader):
            return False, None

    return True, instances[0]


def _migrate_mimic_joint(stage, prim):
    """Convert PhysxMimicJointAPI (single instance) → NewtonMimicAPI on a single-DOF joint."""
    ok, instance = _can_migrate_mimic(prim)
    if not ok:
        return

    api = PhysxSchema.PhysxMimicJointAPI.Get(prim, instance)

    gearing = 1.0
    offset = 0.0
    gearing_attr = api.GetGearingAttr()
    if gearing_attr and gearing_attr.HasAuthoredValue():
        gearing = gearing_attr.Get()
    offset_attr = api.GetOffsetAttr()
    if offset_attr and offset_attr.HasAuthoredValue():
        offset = offset_attr.Get()

    ref_targets = list(api.GetReferenceJointRel().GetTargets())

    prim.ApplyAPI("NewtonMimicAPI")
    prim.GetAttribute("newton:mimicEnabled").Set(True)
    prim.GetAttribute("newton:mimicCoef1").Set(-float(gearing))
    prim.GetAttribute("newton:mimicCoef0").Set(-float(offset))
    prim.GetRelationship("newton:mimicJoint").SetTargets(ref_targets)

    # Remove the PhysxMimicJointAPI schema + its namespaced properties so only the
    # Newton schema remains.
    prim.RemoveAPI(PhysxSchema.PhysxMimicJointAPI, instance)
    ns_prefix = f"physxMimicJoint:{instance}:"
    for prop in list(prim.GetProperties()):
        if prop.GetName().startswith(ns_prefix):
            prim.RemoveProperty(prop.GetName())


@registerRule("Omni:Physx")
class NewtonMimicMigrationChecker(BaseRuleChecker):
    __doc__ = """
    Check for PhysxMimicJointAPI applied to single-DOF joints that can be migrated
    to the single-apply NewtonMimicAPI schema.

    Migration is only offered for joints that meet all of these criteria:
      - exactly one PhysxMimicJointAPI instance applied (Newton is single-apply),
      - follower joint is a PhysicsRevoluteJoint or PhysicsPrismaticJoint,
      - leader joint (``referenceJoint``) is also a PhysicsRevoluteJoint or
        PhysicsPrismaticJoint.

    The sign-flipped mapping preserves the original constraint:
      jointPos + gearing*refPos + offset = 0  ⇔  jointPos = (-offset) + (-gearing)*refPos
    so newton:mimicCoef1 = -gearing and newton:mimicCoef0 = -offset.
    """

    DEPRECATED_MIMIC_API_CODE = "NewtonMimicMigrationChecker:DeprecatedMimicAPI"

    def _fix_prim(self, stage: Usd.Stage, prim: Usd.Prim):
        _migrate_mimic_joint(stage, prim)

    def CheckPrim(self, prim: Usd.Prim):
        ok, instance = _can_migrate_mimic(prim)
        if not ok:
            return

        self._AddFailedCheck(
            message=f"PhysxMimicJointAPI:{instance} on '{prim.GetPath()}' can be migrated to "
                    f"NewtonMimicAPI (single-DOF follower and leader).",
            at=prim,
            suggestion=Suggestion(
                message=f"Migrate PhysxMimicJointAPI:{instance} to NewtonMimicAPI",
                callable=self._fix_prim,
                at=[prim.GetStage().GetRootLayer()],
            ),
            code=self.DEPRECATED_MIMIC_API_CODE,
        )
