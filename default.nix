{
  lib,
  stdenv,
  cmake,
  crocoddyl,
  ipopt,
  pkg-config,
  python3Packages,
  pythonSupport ? false,
}:

stdenv.mkDerivation {
  pname = "colmpc";
  version = "0.2.0";

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

  nativeBuildInputs = [
    cmake
    pkg-config
  ] ++ lib.optional pythonSupport python3Packages.pythonImportsCheckHook;

  propagatedBuildInputs =
    [ ipopt ]
    ++ lib.optional pythonSupport python3Packages.crocoddyl
    ++ lib.optional (!pythonSupport) crocoddyl;

  cmakeFlags = [
    (lib.cmakeBool "BUILD_PYTHON_INTERFACE" pythonSupport)
  ];

  doCheck = true;
  pythonImportsCheck = [ "colmpc" ];

  meta = {
    description = "Collision avoidance for MPC";
    homepage = "https://github.com/agimus-project/colmpc";
    license = lib.licenses.bsd2;
    maintainers = with lib.maintainers; [ nim65s ];
    platforms = lib.platforms.unix;
  };
}
