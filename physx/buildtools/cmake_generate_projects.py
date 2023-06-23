import sys
import os
import glob
import os.path
import shutil
import subprocess
import xml.etree.ElementTree


def packmanExt():
    if sys.platform == 'win32':
        return 'cmd'
    return 'sh'


def cmakeExt():
    if sys.platform == 'win32':
        return '.exe'
    return ''


def filterPreset(presetName):
    winPresetFilter = ['win','switch','crosscompile']
    if sys.platform == 'win32':
        if any(presetName.find(elem) != -1 for elem in winPresetFilter):
            return True
    else:
        if all(presetName.find(elem) == -1 for elem in winPresetFilter):
            return True
    return False

def noPresetProvided():
    global input
    print('Preset parameter required, available presets:')
    presetfiles = []
    for file in glob.glob("buildtools/presets/*.xml"):
        presetfiles.append(file)

    if len(presetfiles) == 0:
        for file in glob.glob("buildtools/presets/public/*.xml"):
            presetfiles.append(file)

    counter = 0
    presetList = []
    for preset in presetfiles:
        if filterPreset(preset):
            presetXml = xml.etree.ElementTree.parse(preset).getroot()
            if(preset.find('user') == -1):
                print('(' + str(counter) + ') ' + presetXml.get('name') +
                    ' <--- ' + presetXml.get('comment'))
                presetList.append(presetXml.get('name'))
            else:
                print('(' + str(counter) + ') ' + presetXml.get('name') +
                    '.user <--- ' + presetXml.get('comment'))
                presetList.append(presetXml.get('name') + '.user')
            counter = counter + 1
    # Fix Python 2.x.
    try:
    	input = raw_input
    except NameError:
    	pass
    mode = int(eval(input('Enter preset number: ')))
    return presetList[mode]

class CMakePreset:
    presetName = ''
    targetPlatform = ''
    compiler = ''
    cmakeSwitches = []
    cmakeParams = []

    def __init__(self, presetName):
        xmlPath = "buildtools/presets/"+presetName+'.xml'
        if os.path.isfile(xmlPath):
            print('Using preset xml: '+xmlPath)
        else:
            xmlPath = "buildtools/presets/public/"+presetName+'.xml'
            if os.path.isfile(xmlPath):
                print('Using preset xml: '+xmlPath)
            else:
                print('Preset xml file: '+xmlPath+' not found')
                exit()

        # get the xml
        presetNode = xml.etree.ElementTree.parse(xmlPath).getroot()
        self.presetName = presetNode.attrib['name']
        for platform in presetNode.findall('platform'):
            self.targetPlatform = platform.attrib['targetPlatform']
            self.compiler = platform.attrib['compiler']
            print('Target platform: ' + self.targetPlatform +
                  ' using compiler: ' + self.compiler)

        for cmakeSwitch in presetNode.find('CMakeSwitches'):
            cmSwitch = '-D' + \
                cmakeSwitch.attrib['name'] + '=' + \
                cmakeSwitch.attrib['value'].upper()
            self.cmakeSwitches.append(cmSwitch)

        for cmakeParam in presetNode.find('CMakeParams'):
            if cmakeParam.attrib['name'] == 'CMAKE_INSTALL_PREFIX' or cmakeParam.attrib['name'] == 'PX_OUTPUT_LIB_DIR' or cmakeParam.attrib['name'] == 'PX_OUTPUT_EXE_DIR' or cmakeParam.attrib['name'] == 'PX_OUTPUT_DLL_DIR':
                cmParam = '-D' + cmakeParam.attrib['name'] + '=\"' + \
                    os.environ['PHYSX_ROOT_DIR'] + '/' + \
                    cmakeParam.attrib['value'] + '\"'
            else:
                cmParam = '-D' + \
                    cmakeParam.attrib['name'] + '=' + \
                    cmakeParam.attrib['value']
            self.cmakeParams.append(cmParam)
    pass

    def isMultiConfigPlatform(self):
        if self.targetPlatform == 'linux':
            return False
        elif self.targetPlatform == 'linuxAarch64':
            return False
        return True

    def getCMakeSwitches(self):
        outString = ''
        # We need gpuProjectsFound flag to avoid issues when we have both
        # PX_GENERATE_GPU_PROJECTS and PX_GENERATE_GPU_PROJECTS_ONLY switches
        gpuProjectsFound = False  # initialize flag
        for cmakeSwitch in self.cmakeSwitches:
            outString = outString + ' ' + cmakeSwitch
            if not gpuProjectsFound and cmakeSwitch.find('PX_GENERATE_GPU_PROJECTS') != -1:
                gpuProjectsFound = True  # set flag to True when keyword found
                if os.environ.get('PM_CUDA_PATH') is not None:
                    outString = outString + ' -DCUDAToolkit_ROOT_DIR=' + \
                            os.environ['PM_CUDA_PATH']
                    if self.compiler in ['vc15', 'vc16', 'vc17']:
                        outString = outString + ' -T cuda=' + os.environ['PM_CUDA_PATH']
                    # TODO: Need to do the same for gcc (aarch64) when we package it with Packman
                    elif self.compiler == 'clang':
                        if os.environ.get('PM_clang_PATH') is not None:
                            outString = outString + ' -DCMAKE_CUDA_HOST_COMPILER=' + \
                                os.environ['PM_clang_PATH'] + '/bin/clang++'
        return outString

    def getCMakeParams(self):
        outString = ''
        for cmakeParam in self.cmakeParams:
            outString = outString + ' ' + cmakeParam # + ' --trace'
        return outString

    def getPlatformCMakeParams(self):
        cmake_modules_root = os.environ['PHYSX_ROOT_DIR'] + '/source/compiler/cmake/modules'
        outString = ' '
        if self.compiler == 'vc15':
            outString = outString + '-G \"Visual Studio 15 2017\"'
        elif self.compiler == 'vc16':
            outString = outString + '-G \"Visual Studio 16 2019\"'
        elif self.compiler == 'vc17':
            outString = outString + '-G \"Visual Studio 17 2022\"'
        elif self.compiler == 'xcode':
            outString = outString + '-G Xcode'
        elif self.targetPlatform == 'linux':
            outString = outString + '-G \"Unix Makefiles\"'
        elif self.targetPlatform == 'linuxAarch64':
            outString = outString + '-G \"Unix Makefiles\"'

        if self.targetPlatform == 'win64':
            outString = outString + ' -Ax64'
            outString = outString + ' -DTARGET_BUILD_PLATFORM=windows'
            outString = outString + ' -DPX_OUTPUT_ARCH=x86'
            return outString
        elif self.targetPlatform == 'switch64':
            outString = outString + ' -DTARGET_BUILD_PLATFORM=switch'
            outString = outString + ' -DCMAKE_TOOLCHAIN_FILE=' + \
                cmake_modules_root + '/switch/NX64Toolchain.txt'
            outString = outString + ' -DCMAKE_GENERATOR_PLATFORM=NX64'
            return outString
        elif self.targetPlatform == 'linux':
            outString = outString + ' -DTARGET_BUILD_PLATFORM=linux'
            outString = outString + ' -DPX_OUTPUT_ARCH=x86'
            if self.compiler == 'clang-crosscompile':
                outString = outString + ' -DCMAKE_TOOLCHAIN_FILE=' + \
                    cmake_modules_root + '/linux/LinuxCrossToolchain.x86_64-unknown-linux-gnu.cmake'
                outString = outString + ' -DCMAKE_MAKE_PROGRAM=' + os.environ.get('PM_MinGW_PATH') + '/bin/mingw32-make.exe'
            elif self.compiler == 'clang':
                if os.environ.get('PM_clang_PATH') is not None:
                    outString = outString + ' -DCMAKE_C_COMPILER=' + \
                        os.environ['PM_clang_PATH'] + '/bin/clang'
                    outString = outString + ' -DCMAKE_CXX_COMPILER=' + \
                        os.environ['PM_clang_PATH'] + '/bin/clang++'
                else:
                    outString = outString + ' -DCMAKE_C_COMPILER=clang'
                    outString = outString + ' -DCMAKE_CXX_COMPILER=clang++'
            return outString
        elif self.targetPlatform == 'linuxAarch64':
            outString = outString + ' -DTARGET_BUILD_PLATFORM=linux'
            outString = outString + ' -DPX_OUTPUT_ARCH=arm'
            if self.compiler == 'clang-crosscompile':
                outString = outString + ' -DCMAKE_TOOLCHAIN_FILE=' + \
                    cmake_modules_root + '/linux/LinuxCrossToolchain.aarch64-unknown-linux-gnueabihf.cmake'
                outString = outString + ' -DCMAKE_MAKE_PROGRAM=' + os.environ.get('PM_MinGW_PATH') + '/bin/mingw32-make.exe'
            elif self.compiler == 'gcc':
                # TODO: To change so it uses Packman's compiler. Then add it as
                # host compiler for CUDA above.
                outString = outString + ' -DCMAKE_TOOLCHAIN_FILE=\"' + \
                    cmake_modules_root + '/linux/LinuxAarch64.cmake\"'
            return outString
        elif self.targetPlatform == 'mac64':
            outString = outString + ' -DTARGET_BUILD_PLATFORM=mac'
            outString = outString + ' -DPX_OUTPUT_ARCH=x86'
            return outString
        return ''


def getCommonParams():
    outString = '--no-warn-unused-cli'
    outString = outString + ' -DCMAKE_PREFIX_PATH=\"' + os.environ['PM_PATHS'] + '\"'
    outString = outString + ' -DPHYSX_ROOT_DIR=\"' + \
        os.environ['PHYSX_ROOT_DIR'] + '\"'
    outString = outString + ' -DPX_OUTPUT_LIB_DIR=\"' + \
        os.environ['PHYSX_ROOT_DIR'] + '\"'
    outString = outString + ' -DPX_OUTPUT_BIN_DIR=\"' + \
        os.environ['PHYSX_ROOT_DIR'] + '\"'
    if os.environ.get('GENERATE_SOURCE_DISTRO') == '1':
        outString = outString + ' -DPX_GENERATE_SOURCE_DISTRO=1'
    return outString

def cleanupCompilerDir(compilerDirName):
    if os.path.exists(compilerDirName):
        if sys.platform == 'win32':
            os.system('rmdir /S /Q ' + compilerDirName)
        else:
            shutil.rmtree(compilerDirName, True)
    if os.path.exists(compilerDirName) == False:
        os.makedirs(compilerDirName)

def presetProvided(pName):
    parsedPreset = CMakePreset(pName)

    print('PM_PATHS: ' + os.environ['PM_PATHS'])

    if os.environ.get('PM_cmake_PATH') is not None:
        cmakeExec = os.environ['PM_cmake_PATH'] + '/bin/cmake' + cmakeExt()
    else:
        cmakeExec = 'cmake' + cmakeExt()
    print('Cmake: ' + cmakeExec)

    # gather cmake parameters
    cmakeParams = parsedPreset.getPlatformCMakeParams()
    cmakeParams = cmakeParams + ' ' + getCommonParams()
    cmakeParams = cmakeParams + ' ' + parsedPreset.getCMakeSwitches()
    cmakeParams = cmakeParams + ' ' + parsedPreset.getCMakeParams()
    # print(cmakeParams)

    if os.path.isfile(os.environ['PHYSX_ROOT_DIR'] + '/compiler/internal/CMakeLists.txt'):
        cmakeMasterDir = 'internal'
    else:
        cmakeMasterDir = 'public'
    if parsedPreset.isMultiConfigPlatform():
        # cleanup and create output directory
        outputDir = os.path.join('compiler', parsedPreset.presetName)
        cleanupCompilerDir(outputDir)

        # run the cmake script
        #print('Cmake params:' + cmakeParams)
        os.chdir(os.path.join(os.environ['PHYSX_ROOT_DIR'], outputDir))
        os.system(cmakeExec + ' \"' +
                  os.environ['PHYSX_ROOT_DIR'] + '/compiler/' + cmakeMasterDir + '\"' + cmakeParams)
        os.chdir(os.environ['PHYSX_ROOT_DIR'])
    else:
        configs = ['debug', 'checked', 'profile', 'release']
        for config in configs:
            # cleanup and create output directory
            outputDir = os.path.join('compiler', parsedPreset.presetName + '-' + config)
            cleanupCompilerDir(outputDir)

            # run the cmake script
            #print('Cmake params:' + cmakeParams)
            os.chdir(os.path.join(os.environ['PHYSX_ROOT_DIR'], outputDir))
            # print(cmakeExec + ' \"' + os.environ['PHYSX_ROOT_DIR'] + '/compiler/' + cmakeMasterDir + '\"' + cmakeParams + ' -DCMAKE_BUILD_TYPE=' + config)
            os.system(cmakeExec + ' \"' + os.environ['PHYSX_ROOT_DIR'] + '/compiler/' +
                      cmakeMasterDir + '\"' + cmakeParams + ' -DCMAKE_BUILD_TYPE=' + config)
            os.chdir(os.environ['PHYSX_ROOT_DIR'])
    pass


def main():
    if (sys.version_info[0] < 3) or (sys.version_info[0] == 3 and sys.version_info[1] < 5):
        print("You are using Python {}. You must use Python 3.5 and up. Please read README.md for requirements.").format(sys.version)
        exit()
    physx_root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
    os.environ['PHYSX_ROOT_DIR'] = physx_root_dir.replace("\\", "/")
    if len(sys.argv) != 2:
        presetName = noPresetProvided()
        if sys.platform == 'win32':
            print('Running generate_projects.bat ' + presetName)
            cmd = 'generate_projects.bat {}'.format(presetName)
            result = subprocess.run(cmd, cwd=os.environ['PHYSX_ROOT_DIR'], check=True, universal_newlines=True)
            # TODO: catch exception and add capture errors
        else:
            print('Running generate_projects.sh ' + presetName)
            # TODO: once we have Python 3.7.2 for linux, add the text=True instead of universal_newlines
            cmd = './generate_projects.sh {}'.format(presetName)
            result = subprocess.run(['bash', './generate_projects.sh', presetName], cwd=os.environ['PHYSX_ROOT_DIR'], check=True, universal_newlines=True)
            # TODO: catch exception and add capture errors
    else:
        presetName = sys.argv[1]
        if filterPreset(presetName):
            presetProvided(presetName)
        else:
            print('Preset not supported on this build platform.')

main()
