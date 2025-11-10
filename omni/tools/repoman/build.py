# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import os
import platform
import atexit

import logging

import xml.etree.ElementTree as ET

import omni.repo.man
import omni.repo.build

logger = logging.getLogger(os.path.basename(__file__))

root = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..")
depFileXml = '<project toolsVersion="5.6"></project>'
scriptDir = os.path.dirname(os.path.realpath(__file__))
rootDir = os.path.join(scriptDir, os.path.normcase("../.."))
blokyUserDepsDir = os.path.join(rootDir, "deps")
blokyTargetDepsFile = os.path.join(blokyUserDepsDir, "target-deps.packman.xml")
blokyUserDepsFile = os.path.join(blokyUserDepsDir, "target-deps.packman.xml.user")
physxDir = os.path.realpath(os.path.join(rootDir, os.path.normcase("../physx")))


def clean():
    logger.info("cleaning repo:")
    folders = ["_build", "_compiler", "_builtpackages", "_ovat", "_ovatdev"]

    root = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..")
    for folder in folders:
        folder = os.path.abspath(os.path.join(root, folder))
        # having to do the platform check because its safer when you might be removing
        # folders with windows junctions.
        if os.path.exists(folder):
            logger.info(f"removing folder: {folder}")
            if platform.system() == "Windows":
                os.system("rmdir /q /s %s > nul 2>&1" % folder)
            else:
                os.system("rm -r -f %s > /dev/null 2>&1" % folder)
            if os.path.exists(folder):
                logger.warn(
                    f"{folder} was not successfully removed, most probably due to a file lock on 1 or more of the files."
                )


class CommentedTreeBuilder(ET.TreeBuilder):
    def comment(self, data):
        self.start(ET.Comment, {})
        self.data(data)
        self.end(ET.Comment)


def loadUserDepFile(filename):
    if os.path.exists(filename):  
        # convert non breaking spaces to whitespaces first
        newLines = []
        with open(filename, 'r') as fp:
            lines = fp.readlines()            
            for line in lines:
                newLines.append(line.replace("\xc2\xa0", " "))      
            fp.close()

        with open(filename, 'w') as fp:
            fp.writelines(newLines)
            fp.close()

        parser = ET.XMLParser(target=CommentedTreeBuilder())
        return ET.parse(filename, parser)
    else:
        return ET.ElementTree(ET.fromstring(depFileXml))


def removeXmlDep(root, name):
    for dep in root.findall("dependency"):
        if dep.get("name") == name:
            root.remove(dep)
            return


def updateXmlDep(root, name, sourcePath):
    # Remove the element first
    removeXmlDep(root, name)

    # Now add it
    dep = ET.SubElement(root, "dependency")
    dep.set("name", name)
    dep.set("linkPath", "../_build/target-deps/" + name)
    src = ET.SubElement(dep, "source")    
    src.set("path", sourcePath)


def setupUserDepFile(sourcePath):
    xmlTree = loadUserDepFile(blokyUserDepsFile)
    root = xmlTree.getroot()
    updateXmlDep(root, "physx", sourcePath)
    xmlTree.write(blokyUserDepsFile)


def removeUserDepFile():
    if os.path.isfile(blokyUserDepsFile):
        xmlTree = loadUserDepFile(blokyUserDepsFile)
        root = xmlTree.getroot()
        removeXmlDep(root, "physx")
        xmlTree.write(blokyUserDepsFile)


def before_run_callback(options, platform_host, settings):
    if not options.userphysx:
        removeUserDepFile()

    if options.devphysx:
        if options.userphysx:
            print("WARNING --userphysx takes precedence over --devphysx!")
        else:
            scriptPath = os.path.realpath(os.path.join(options.devphysx, os.path.normcase("generate_projects.bat")))
            if not os.path.exists(options.devphysx) or not os.path.exists(scriptPath):
                print(f"ERROR --devphysx path '{options.devphysx}' does not exists or generate_projects is missing in that dir)!")
                return False

            print(f"Using PhysX SDK from source: {options.devphysx}")
            settings["repo_build"]["premake"]["extra_args"].append(f"--devphysx={options.devphysx}")
            setupUserDepFile(options.devphysx)

    if options.nopch:
        print("Disabling precompiled headers.")
        settings["repo_build"]["premake"]["extra_args"].append("--nopch")

    if options.notensors:
        print("Skipping tensor API extensions.")
        settings["repo_build"]["premake"]["extra_args"].append("--notensors")

    if options.devschema:
        print("Using local physics schema.")
        settings["repo_build"]["premake"]["extra_args"].append("--devschema")
    else:
        repo_root = settings["repo"]["folders"]["root"]
        schema_dep = f"{repo_root}/deps/schema-deps.packman.xml"
        print(f"Added {schema_dep} to packman_target_files_to_pull.")
        settings["repo_build"]["fetch"]["packman_target_files_to_pull"].append(schema_dep)

    if options.nosymbols:
        print("Building with no debug info.")
        settings["repo_build"]["premake"]["extra_args"].append("--no-symbols")

    return True


def setup_repo_tool(parser, config):
    omni.repo.build.setup_repo_tool(parser, config)

    parser.add_argument(
        "--devphysx",
        dest="devphysx",
        nargs="?",
        const=f"{physxDir}",
        required=False,
        help="Enable support for PhysX SDK development, include PhysX SDK solution into bloky solution.",
    )

    parser.add_argument(
        "--devschema",
        dest="devschema",
        action="store_true",
        required=False,
        help="Enable schema development, run schemagen.",
    )

    parser.add_argument(
        "--nopch",
        dest="nopch",
        action="store_true",
        required=False,
        help="Disable precompiled headers.",
    )

    parser.add_argument(
        "--notensors",
        dest="notensors",
        action="store_true",
        required=False,
        help="Don't build tensor API extensions. A HACK to work around CUDA build bug.",
    )

    parser.add_argument(
        "--userphysx",
        dest="userphysx",
        action="store_true",
        required=False,
        help="PhysX SDK source code provided in target-deps.xml.user file.",
    )

    parser.add_argument(
        "--no-symbols",
        dest="nosymbols",
        action="store_true",
        required=False,
        help="",
    )

    def run_repo_tool(options, config):
        repo_folders = config["repo"]["folders"]

        omni.repo.man.open_teamcity_block("Building", f"Kit {os.getenv('BUILD_NUMBER', '0')}")
        atexit.register(omni.repo.man.close_teamcity_block, "Building")

        if not before_run_callback(options, omni.repo.man.get_host_platform(), config):
            return run_repo_tool

        omni.repo.build.run_build(options, repo_folders, None, config)

    return run_repo_tool
