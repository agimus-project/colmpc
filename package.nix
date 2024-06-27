{
  lib,
  buildPythonPackage,
  cmake,
  jrl-cmakemodules,
  mim-solvers,
  py-mim-solvers,
  pythonSupport ? false,
}:

buildPythonPackage {
  pname = "colmpc";
  version = "0.1.0";
  pyproject = false;

  src = lib.fileset.toSource {
    root = ./.;
    fileset = lib.fileset.unions [
      ./CMakeLists.txt
      ./include
      ./package.xml
      ./python
      ./tests
    ];
  };

  cmakeFlags = [ (lib.cmakeBool "BUILD_PYTHON_INTERFACE" pythonSupport) ];

  strictDeps = true;

  nativeBuildInputs = [ cmake ];
  propagatedBuildInputs =
    [ jrl-cmakemodules ]
    ++ lib.optionals (!pythonSupport) [ mim-solvers ] ++ lib.optionals pythonSupport [ py-mim-solvers ];

  pythonImportsCheck = [ "colmpc" ];

  meta = {
    description = "Collision avoidance for MPC";
    homepage = "https://github.com/agimus-project/colmpc";
    license = lib.licenses.bsd2;
    maintainers = [ lib.maintainers.nim65s ];
  };
}
