# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pathlib import Path
import re
import sys
import os

# Add the physxSchema directory directly to the path
physics_schema_tools = Path(__file__).parent.parent.joinpath("source", "physicsSchemaTools")
sys.path.append(str(physics_schema_tools))
from units import UnitParameters


# The generated python dictionary.
python_dictionary_text = """

usd_db_schemas = {"""


attribute_to_schemas = {}

attribute_units = {}

# Valid attribute types in the schema files.
valid_attribute_types = ("token", "rel", "bool", "int", "uint", "float", "vector", "point", "quat")

# Recognized unit names and their corresponding UnitParameters enum values.
unit_name_to_type = {
    "mass": UnitParameters.MASS,
    "distance": UnitParameters.DISTANCE,
    "DIST_UNITS": UnitParameters.DISTANCE,
    "degree": UnitParameters.DEGREES,
    "degrees": UnitParameters.DEGREES,
    "second": UnitParameters.SECONDS,
    "seconds": UnitParameters.SECONDS,
    "time": UnitParameters.SECONDS,
    "radians": UnitParameters.RADIANS,
}

# Map instance names to standardized variant names.
attribute_instance_to_standard_variant = {
    "rotX": "angular",
    "rotY": "angular",
    "rotZ": "angular",
    "angular": "angular",
    "transX": "linear",
    "transY": "linear",
    "transZ": "linear",
    "linear": "linear",
    "distance": "linear",
}

# Map attribute unit variants to standardized variant names.
re_attribute_variant_to_standard_variant = {
    "angular": re.compile("\A(angular|rotation)", re.IGNORECASE),
    "linear": re.compile("\A(linear|translation)", re.IGNORECASE),
}

# Overrides the default unit exponents for specific attributes.
attribute_exponents_overrides = {
    "physics:centerOfMass": (0, 0, 0, 0, 0),  # Always in local space, so effectively unitless.
    "physics:damping": { 'linear': { 'Force': (1, 0, 0, -1, 0), 'Acceleration': (0, 0, 0, -1, 0) },
                         'angular': { 'Force': (1, 2, -1, -1, 0), 'Acceleration': (0, 2, -1, -1, 0) } },
    "physics:stiffness": { 'linear': { 'Force': (1, 0, 0, -2, 0), 'Acceleration': (0, 0, 0, -2, 0) },
                         'angular': { 'Force': (1, 2, -1, -2, 0), 'Acceleration': (0, 2, -1, -2, 0) } },
}

# Overrides the allowed instance names for specific classes. 
# Necessary because schemas do not list apiSchemaAllowedInstanceNames for these.
schema_allowed_instance_names_overrides = {
    "LimitAPI": ("transX", "transY", "transZ", "distance", "rotX", "rotY", "rotZ"), 
    "DriveAPI": ("transX", "transY", "transZ", "rotX", "rotY", "rotZ", "angular", "linear"),
    "PhysxLimitAPI": ("transX", "transY", "transZ", "rotX", "rotY", "rotZ", "distance", "angular", "linear"),
    "JointStateAPI": ("transX", "transY", "transZ", "rotX", "rotY", "rotZ", "distance", "angular", "linear"),
    "PhysxMimicJointAPI": ("rotX", "rotY", "rotZ"), 
    "PhysxVehicleBrakesAPI": ("brakes0", "brakes1"),
    "PhysxVehicleNonlinearCommandResponseAPI": ("brakes0", "brakes1"),
}

# Regular expressions for parsing the schema files.
re_class_pattern = re.compile("\nclass .*\"(.*)\"")
re_class_header_pattern = re.compile("\n\((.*)\n\)", re.DOTALL)
re_class_header_name_pattern = re.compile("\n[ \t]*string[ \t]*className[ \t]*=[ \t]*\"(.*)\"")
re_class_header_multiple_apply_pattern = re.compile("\n[ \t]*token[ \t]*apiSchemaType[ \t]*=[ \t]*\"multipleApply\"")
re_class_header_namespace_prefix_pattern = re.compile("\n[ \t]*token[ \t]*propertyNamespacePrefix[ \t]*=[ \t]*\"(.*)\"")
re_class_header_allowed_instance_names_pattern = re.compile("\n[ \t]*token[][ \t]*apiSchemaAllowedInstanceNames[ \t]*=[ \t]*\[(.*)]")
re_class_body_pattern = re.compile("\n{(.*)\n}", re.DOTALL)
re_attribute_pattern = re.compile("\n[ \t]*(?:uniform|)[ \t]*(" + "|".join(valid_attribute_types)+")[\[\]0-9df]*[ \t]+([^ \t=]*)[ \t=]")
re_attribute_doc_units_pattern = re.compile("\n[ \t]*doc[ \t]*=[ \t\n]*\"\"\".*[Uu]nits:(.*)\"\"\"", re.DOTALL)
re_attribute_doc_units_param_pattern = re.compile(r"(\*|/|\^[ \t]?[0-9]+|\. |: |, |=| - |[a-zA-Z0-9,_-]+)[ \t]*")

def parse_usda_schema(file_path) :
    print(f"Parsing {file_path}")

    class_name = ""
    attribute_name = ""
    units = ""

    with open(file_path, 'r') as file:
        file_data = file.read()
        class_matches = re_class_pattern.finditer(file_data)
        class_match_next = next(class_matches, None)
        while class_match_next:
            class_dictionary = ""
            class_name = class_match_next.group(1)
            class_begin = class_match_next.span()[1]

            # Find the next class definition. This allows us to limit parsing of the current class to this point.
            class_match_next = next(class_matches, None)
            if class_match_next:
                class_next_begin = class_match_next.span()[0]
            else:
                class_next_begin = len(file_data)

            re_match_class_header = re_class_header_pattern.search(file_data, class_begin, class_next_begin)
            if not re_match_class_header:
                print(f"Error: No class header found for class {class_name}")
                continue
            
            # If a class name is specified in the header, use it.
            re_match_class_header_name = re_class_header_name_pattern.search(file_data, *re_match_class_header.span(1))
            if re_match_class_header_name:
                class_name = re_match_class_header_name.group(1)

            re_match_class_header_multiple_apply = re_class_header_multiple_apply_pattern.search(file_data, *re_match_class_header.span(1))
            if re_match_class_header_multiple_apply:
                multiple_apply = True
            else:
                multiple_apply = False

            if multiple_apply:
                re_match_class_header_namespace_prefix = re_class_header_namespace_prefix_pattern.search(file_data, *re_match_class_header.span(1))
                if re_match_class_header_namespace_prefix:
                    class_namespace_prefix = re_match_class_header_namespace_prefix.group(1)
                else:
                    class_namespace_prefix = ""

                allowed_instance_names = schema_allowed_instance_names_overrides.get(class_name, None)
                if allowed_instance_names is None:
                    re_match_class_header_allowed_instance_names = re_class_header_allowed_instance_names_pattern.search(file_data, *re_match_class_header.span(1))
                    if re_match_class_header_allowed_instance_names:
                        allowed_instance_names = re_match_class_header_allowed_instance_names.group(1).replace(" ", "").replace("\"", "").split(",")

                if allowed_instance_names is None:
                    allowed_instance_names = []


            re_match_class_body = re_class_body_pattern.search(file_data, class_begin, class_next_begin)
            if not re_match_class_body:
                print(f"Error: No class body found for class {class_name}")
                continue

            attribute_matches = re_attribute_pattern.finditer(file_data, *re_match_class_body.span(1))
            attribute_match_next = next(attribute_matches, None)

            while attribute_match_next:
                attribute_type = attribute_match_next.group(1)
                attribute_name = attribute_match_next.group(2)
                attribute_begin = attribute_match_next.span(2)[1]
                attribute_match_next = next(attribute_matches, None)
                if(attribute_type == "token" or attribute_type == "rel" or attribute_type == "bool" or 
                    attribute_type == "int" or attribute_type == "uint"):
                    continue

                if attribute_match_next:
                    attribute_next_begin = attribute_match_next.span(2)[0]
                else:
                    attribute_next_begin = re_match_class_body.span(1)[1]
                
                units_params_variants = {}
                exponents = attribute_exponents_overrides.get(attribute_name, None)
                if exponents is not None:
                    if type(exponents) is dict:
                        units_params_variants = exponents
                    else:
                        units_params_variants["default"] = exponents
                else:
                    # Find the units text within the doc section of the attribute.
                    re_match_doc_units = re_attribute_doc_units_pattern.search(file_data, attribute_begin, attribute_next_begin)

                    if not re_match_doc_units:
                        continue

                    units = re_match_doc_units.group(1)
                    # Now parse the units from the unit section. Note that some may have multiple variants, in which case we
                    # the unit parameters will be stored in a dictionary with entries for each variant.
                    units_lines = units.splitlines()

                    for line in units_lines:
                        units_params = None
                        unit_variant = ""
                        # The index of the last identified parameter.
                        current_param_index = -1
                        # The current operator used for the following parameter.
                        current_param_operator = "*"
                        # Assume that the beginning of the line is the variant type name.
                        parsing_variant_type_name = True

                        params = re_attribute_doc_units_param_pattern.findall(line)
                        for param in params:
                            param = param.strip()
                            if param.lower() == "unitless" or param.lower() == "dimensionless":
                                units_params_variants[unit_variant if unit_variant != "" else "default"] = tuple([0] * len(UnitParameters))
                                break
                            elif param == "1":
                                # Special case for when starting with e.g. 1 / 
                                # Use enumerator length to identify this.
                                current_param_index = len(UnitParameters)
                                continue
                            elif param in unit_name_to_type.keys():
                                # If this is a recognized unit name, set it as the current parameter.
                                # NB. don't continue or break here as this will be further processed below.
                                current_param_index = unit_name_to_type[param]
                            elif param == ".":
                                break
                            elif current_param_index != -1:
                                if param == "*" or param == "/":
                                    current_param_operator = param
                                elif param[0] == "^":
                                    exponent = int(param[1:])
                                    if units_params[current_param_index] < 0:
                                        units_params[current_param_index] -= exponent -1
                                    else:
                                        units_params[current_param_index] += exponent -1
                                elif param == "=":
                                    # Assume that the following input will alias the current parameters, so reset.
                                    for unit_param in range(len(units_params)):
                                        units_params[unit_param] = 0
                                    current_param_operator = "*"
                                    current_param_index = -1
                                elif param.lower() == "or" and unit_variant == "":
                                    # Special case handling. Assume angular if degrees are set, otherwise linear.
                                    if units_params[UnitParameters.DEGREES] != 0:
                                        units_params_variants["angular"] = tuple(units_params)
                                        unit_variant = "linear"
                                    else:
                                        units_params_variants["linear"] = tuple(units_params)
                                        unit_variant = "angular"
                                    units_params = None
                                    current_param_operator = "*"
                                    current_param_index = -1
                                else:
                                    break
                                continue
                            elif not parsing_variant_type_name:
                                continue
                            else:
                                if param == ":" or param == "-":
                                    parsing_variant_type_name = False
                                    continue
                                elif param.lower() == "if":
                                    continue
                                elif unit_variant == "":
                                    unit_variant = param
                                else:
                                    unit_variant += " " + param
                                continue

                            if units_params is None:
                                units_params = [0] * len(UnitParameters)

                            if current_param_operator == "/":
                                units_params[current_param_index] -= 1
                            elif current_param_operator == "*":
                                units_params[current_param_index] += 1
                        
                        if units_params is None:
                            if len(units_params_variants) > 0:
                                # We've set no new unit params for this line. If we've already set some, assume that we are done.
                                break
                        else:
                            units_params_variants[unit_variant if unit_variant != "" else "default"] = tuple(units_params)

                if(len(units_params_variants) > 0):
                    class_dictionary += f"            '{attribute_name}': "

                    # Only add a variant dictionary if there are multiple variants.
                    if len(units_params_variants) > 1:
                        class_dictionary += "{\n"
                        for name, value in units_params_variants.items():
                            class_dictionary += "                '" + name + "': " + f"{value}" + ",\n"
                        class_dictionary += "            },\n"
                    else:
                        units_params = next(iter(units_params_variants.values()))
                        class_dictionary += f"{units_params}" + ",\n"

                    def register_attribute_exponents(attribute, exponents):
                        if attribute not in attribute_units:
                            attribute_units[attribute] = exponents
                        else:
                            # If there are already an entry for this attribute, we need to merge the new unit params with the existing ones.
                            current_units = attribute_units[attribute]
                            if type(current_units) is not dict:
                                current_schema_name = attribute_to_schemas[attribute][0]
                                current_units = {current_schema_name: current_units}
                                attribute_units[attribute] = current_units

                            if type(exponents) is dict:
                                current_units.update(exponents)
                                attribute_units[attribute] = current_units
                            else:
                                current_units.update({class_name: exponents})
                                attribute_units[attribute] = current_units

                        schema_names = attribute_to_schemas.get(attribute, None)
                        if schema_names is None:
                            schema_names = (class_name,)
                        else:
                            schema_names = schema_names + (class_name,)
                        attribute_to_schemas[attribute] = schema_names

                    if multiple_apply and len(allowed_instance_names) > 0:
                        # If the class is multiple apply, we need to register the attribute for each instance name.
                        for instance_name in allowed_instance_names:
                            entry = (class_namespace_prefix + ":" if class_namespace_prefix != "" else "") + instance_name + ":" + attribute_name
                            instance_standard_variant = attribute_instance_to_standard_variant.get(instance_name, None)
                            if len(units_params_variants) > 1:
                                # If there are multiple variants, attempt to match the instance name to a variant.
                                if instance_standard_variant is not None:
                                    re_variant = re_attribute_variant_to_standard_variant[instance_standard_variant]
                                    for variant, units_params in units_params_variants.items():
                                        if re_variant.match(variant):
                                            register_attribute_exponents(entry, units_params)
                                            found_variant = True
                                            break
                                if not found_variant:
                                    register_attribute_exponents(entry, units_params_variants)
                            else:
                                register_attribute_exponents(entry, units_params)
                    else:
                        if len(units_params_variants) > 1:
                            register_attribute_exponents(attribute_name, units_params_variants)
                        else:
                            register_attribute_exponents(attribute_name, units_params)
                else:
                    print(f"Warning: failed to parse units for attribute {class_name}:{attribute_name}")

            if class_dictionary != "":
                class_properties = "\n    '" + class_name + "': {\n"                  \
                    + "        'multiple_apply': " +str(multiple_apply) + ",\n"
                if multiple_apply:
                    class_properties += "        'namespace_prefix': \""+class_namespace_prefix+ "\",\n"
                    if len(allowed_instance_names) > 0:
                        class_properties += "        'allowed_instance_names': ('" + "', '".join(allowed_instance_names) + "'),\n"

                global python_dictionary_text
                python_dictionary_text += class_properties + "        'attributes': {\n"  \
                    + class_dictionary + "        },\n    },\n" \


print("Generating python unit dictionary from schemas...")

if len(sys.argv) < 2:
    print("Error: no input path for data file. Aborting.")
else:
    parse_usda_schema(Path(__file__).parent.parent.joinpath("source", "physxSchema", "schema.usda"))
    parse_usda_schema(Path(__file__).parent.parent.joinpath("_build", "target-deps", "usd", "release", "lib", "usd", "usdPhysics", "resources", "usdPhysics", "schema.usda"))

    python_dictionary_text += """\n}\n"""

    attribute_to_schemas_text = f"usd_db_attribute_to_schemas = {attribute_to_schemas}"
    attribute_to_schemas_text = attribute_to_schemas_text.replace("),", "),\n   ")

    attributes_text = "usd_attribute_units = {\n"
    for entry, units in attribute_units.items():
        units_text = ""
        if isinstance(units, dict):
            if len(set(units.values())) > 1:
                print(f"Warning: attribute {entry} is ambiguous and must be resolved upon query. Variants: {tuple(units.keys())}")
                units_text = "{\n"
                for variant, units_params in units.items():
                    units_text += f"        '{variant}': {tuple(units_params)},\n"
                units_text += "    },\n"
            else:
                units_text += f"{next(iter(units.values()))},\n"
        else:
            units_text += f"{units},\n"
        attributes_text += f"    '{entry}': {units_text}"
    attributes_text += "}"

    if not os.path.exists(sys.argv[1]):
        os.makedirs(sys.argv[1])
    with open(Path(sys.argv[1]).joinpath("units_db.py"), "w") as file:
        file.write(python_dictionary_text + "\n\n" + attribute_to_schemas_text + "\n\n" + attributes_text)

    print("Done.")
