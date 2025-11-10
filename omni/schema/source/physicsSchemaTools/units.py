from enum import IntEnum, auto
import math
try:
    from .units_db import usd_attribute_units
except ImportError:
    # This should only happen if the schema database has not been generated yet.
    usd_attribute_units = {}

try:
    from pxr import UsdPhysics, PhysxSchema, UsdGeom
    from carb import log_warn
except ImportError:
    pass


# Enum representing the different unit types in the USD schemas.
class UnitParameters(IntEnum):
    MASS = 0
    DISTANCE = auto()
    DEGREES = auto()
    SECONDS = auto()
    RADIANS = auto()


# Returns the unit exponents for a given attribute as a tuple of (mass, distance, degrees, seconds, radians).
# The exponents indicate the power of each unit type in the attribute's units.
# For example, velocity would have distance=1, seconds=-1 for units of m/s
def get_attribute_unit_exponents(attribute_name, prim=None) -> tuple[int, int, int, int, int] | dict[str, tuple[int, int, int, int, int]] | None:
    exponents = usd_attribute_units.get(attribute_name, None)
    if type(exponents) is not dict:
        return exponents
    
    if prim is None:
        return exponents

    # If a prim is provided and the attribute has multiple variants, attempt to determine which one to use.
    match(attribute_name):
        case "physics:lowerLimit" | "physics:upperLimit":
            if prim.IsA(UsdPhysics.RevoluteJoint):
                return exponents["RevoluteJoint"]
            else:
                return exponents["PrismaticJoint"]
        case "physxJoint:maxJointVelocity" | "physxJoint:armature":
            if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.SphericalJoint):
                return exponents["Angular joint"]
            else:
                return exponents["Linear joint"]
        case "physxMaterial:compliantContactStiffness" | "physxMaterial:compliantContactDamping":
            if prim.HasAPI(UsdPhysics.MaterialAPI):
                if prim.GetAttribute("physxMaterial:compliantContactAccelerationSpring").Get() == True:
                    return exponents["Acceleration"]
            return exponents["Force"]
        case "physxForce:force" | "physxForce:torque":
            mode = "acceleration"
            if prim.HasAPI(PhysxSchema.PhysxForceAPI):
                mode = PhysxSchema.PhysxForceAPI(prim).GetModeAttr().Get()
            if mode == "force":
                return exponents["Force"]
            else:
                return exponents["Acceleration"]
        case "drive:rotY:physics:stiffness" | "drive:rotX:physics:stiffness" | "drive:rotZ:physics:stiffness" | "drive:angular:physics:stiffness" \
            | "drive:linear:physics:stiffness" | "drive:transX:physics:stiffness" | "drive:transY:physics:stiffness" | "drive:transZ:physics:stiffness" \
            | "drive:rotY:physics:damping" | "drive:rotX:physics:damping" | "drive:rotZ:physics:damping" | "drive:angular:physics:damping" \
            | "drive:linear:physics:damping" | "drive:transX:physics:damping" | "drive:transY:physics:damping" | "drive:transZ:physics:damping":
            attr_type_name = "drive:" + attribute_name.split(":")[1] + ":physics:type"
            attribute = prim.GetAttribute(attr_type_name)
            mode = attribute.Get() if attribute.IsValid() else None
            if mode == UsdPhysics.Tokens.force:
                return exponents["Force"]
            else:
                return exponents["Acceleration"]
            
        case _:
            log_warn(f"get_attribute_unit_exponents failed to determine exponents for {prim} attribute {attribute_name}")
            return None


# Returns the units for a given attribute scaled to the stage's meters per unit and kilograms per unit.
# The first string of the tuple is the symbolized units, the second is the text representation of the units.
# For example, velocity would return ("m/s", "Units: meters / seconds"), and acceleration would return ("m/s²", "Units: meters / seconds²")
# If the attribute units have multiple variants and the relevant variant cannot be determined, the first string will be empty and the second will be the units for each variant.
def get_attribute_units_as_text(attribute_name, stage=None, prim=None) -> tuple[str, str]:
    exponents = get_attribute_unit_exponents(attribute_name, prim)

    if exponents is None:
        return ("", "")
    elif exponents == (0, 0, 0, 0, 0):
        return ("", "Units: Unitless")

    if type(exponents) is not dict:
        exponents = {"default": exponents}

    def exponent_to_string(exponent):
        match exponent:
            case 1:
                return ""
            case 2:
                return "²"
            case 3:
                return "³"
            case _:
                return f"^{exponent}"

    base_multiplier = 1.0
    if stage is not None or prim is not None:
        if stage is None:
            stage = prim.GetStage()
        mpu = UsdGeom.GetStageMetersPerUnit(stage)
        kpu = UsdPhysics.GetStageKilogramsPerUnit(stage)
    else:
        mpu = 1.0
        kpu = 1.0


    units_text = "Units: "
    multipliers_symbolized = ""
    divisors_symbolized = ""

    symbolize = len(exponents) == 1

    for variant_name, variant_exponents in exponents.items():
        variant_multipliers = ""
        variant_divisors = ""

        exp_mass = variant_exponents[UnitParameters.MASS]
        if exp_mass != 0:
            decimal_places = round(math.log(kpu, 10), 1)
            mass_modifier = 1.0
            if decimal_places >= 3:
                mass_unit = ("t", "tonnes")
                mass_modifier = kpu / 1000.0
            elif decimal_places >= 0:
                mass_unit = ("kg", "kilograms")
                mass_modifier = kpu
            elif decimal_places >= -3:
                mass_unit = ("g", "grams")
                mass_modifier = kpu * 1000.0
            elif decimal_places >= -6:
                mass_unit = ("mg", "milligrams")
                mass_modifier = kpu * 1000000.0
            elif decimal_places >= -9:
                mass_unit = ("µg", "micrograms")
                mass_modifier = kpu * 1000000000.0
            elif decimal_places >= -12:
                mass_unit = ("ng", "nanograms")
                mass_modifier = kpu * 1000000000000.0

            mass_modifier = mass_modifier ** exp_mass
            base_multiplier *= mass_modifier
            if exp_mass > 0:
                variant_multipliers = mass_unit[1] + exponent_to_string(exp_mass)
                if symbolize:
                    multipliers_symbolized = mass_unit[0] + exponent_to_string(exp_mass)
            elif exp_mass < 0:
                variant_divisors = " / " + mass_unit[1] + exponent_to_string(-exp_mass)
                if symbolize:
                    divisors_symbolized = "/" + mass_unit[0] + exponent_to_string(-exp_mass)
        
        exp_distance = variant_exponents[UnitParameters.DISTANCE]
        if exp_distance != 0:
            decimal_places = round(math.log(mpu, 10), 1)
            distance_modifier = 1.0
            if decimal_places >= 3:
                distance_unit = ("km", "kilometers")
                distance_modifier = mpu / 1000.0
            elif decimal_places >= 0:
                distance_unit = ("m", "meters")
                distance_modifier = mpu
            elif decimal_places >= -2:
                distance_unit = ("cm", "centimeters")
                distance_modifier = mpu * 100.0
            elif decimal_places >= -3:
                distance_unit = ("mm", "millimeters")
                distance_modifier = mpu * 1000.0
            elif decimal_places >= -6:
                distance_unit = ("µm", "micrometers")
                distance_modifier = mpu * 1000000.0
            else:
                distance_unit = ("nm", "nanometers")
                distance_modifier = mpu * 1000000000.0
        
            distance_modifier = distance_modifier ** exp_distance
            base_multiplier *= distance_modifier
            if exp_distance > 0:
                variant_multipliers += (" * " if variant_multipliers != "" else "") + distance_unit[1] + exponent_to_string(exp_distance)
                if symbolize:
                    multipliers_symbolized += ("·" if multipliers_symbolized != "" else "") + distance_unit[0] + exponent_to_string(exp_distance)
            elif exp_distance < 0:
                variant_divisors += " / " + distance_unit[1] + exponent_to_string(-exp_distance)
                if symbolize:
                    divisors_symbolized += "/" + distance_unit[0] + exponent_to_string(-exp_distance)

        if variant_exponents[UnitParameters.DEGREES] > 0:
            variant_multipliers += (" * " if variant_multipliers != "" else "") + "degrees" + exponent_to_string(variant_exponents[UnitParameters.DEGREES])
            if symbolize:
                multipliers_symbolized += ("·" if multipliers_symbolized != "" else "") + "deg" + exponent_to_string(variant_exponents[UnitParameters.DEGREES])
        elif variant_exponents[UnitParameters.DEGREES] < 0:
            variant_divisors += " / degrees" + exponent_to_string(-variant_exponents[UnitParameters.DEGREES])
            if symbolize:
                divisors_symbolized += "/deg" + exponent_to_string(-variant_exponents[UnitParameters.DEGREES])

        if variant_exponents[UnitParameters.SECONDS] > 0:
            variant_multipliers += (" * " if variant_multipliers != "" else "") + "seconds" + exponent_to_string(variant_exponents[UnitParameters.SECONDS])
            if symbolize:
                multipliers_symbolized += ("·" if multipliers_symbolized != "" else "") + "s" + exponent_to_string(variant_exponents[UnitParameters.SECONDS])
        elif variant_exponents[UnitParameters.SECONDS] < 0:
            variant_divisors += " / seconds" + exponent_to_string(-variant_exponents[UnitParameters.SECONDS])
            if symbolize:
                divisors_symbolized += "/s" + exponent_to_string(-variant_exponents[UnitParameters.SECONDS])

        if variant_exponents[UnitParameters.RADIANS] > 0:
            variant_multipliers += (" * " if variant_multipliers != "" else "") + "radians" + exponent_to_string(variant_exponents[UnitParameters.RADIANS])
            if symbolize:
                multipliers_symbolized += ("·" if multipliers_symbolized != "" else "") + "rad" + exponent_to_string(variant_exponents[UnitParameters.RADIANS])
        elif variant_exponents[UnitParameters.RADIANS] < 0:
            variant_divisors += " / radians" + exponent_to_string(-variant_exponents[UnitParameters.RADIANS])
            if symbolize:
                divisors_symbolized += "/rad" + exponent_to_string(-variant_exponents[UnitParameters.RADIANS])

        if variant_exponents == (0, 0, 0, 0, 0):
            variant_multipliers = "Unitless"
        elif base_multiplier != 1.0:
            variant_multipliers = f"{base_multiplier:g}" + ((" * " + variant_multipliers) if variant_multipliers != "" else "")
            if symbolize:
                multipliers_symbolized = f"{base_multiplier:g}" + (("·" + multipliers_symbolized) if multipliers_symbolized != "" else "")
        elif variant_multipliers == "":
            variant_multipliers = "1"
            if symbolize:
                multipliers_symbolized = "1"

        units_text += ("\n\t" + variant_name + ": " if len(exponents) > 1 else "") + variant_multipliers + variant_divisors

    return multipliers_symbolized + divisors_symbolized, units_text