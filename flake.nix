{
  description = "Testing gepetto softwares";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs =
    { nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs { localSystem = system; };
        hpp-fcl = pkgs.hpp-fcl.overrideAttrs {
          src = pkgs.fetchFromGitHub {
            owner = "humanoid-path-planner";
            repo = "hpp-fcl";
            rev = "65fb435b44a1bbd5059347d7a311cc7c7aa1349e";
            fetchSubmodules = true;
            hash = "sha256-qW5/rRvs0ZPWm9xQE+vf2cLc+0bd9KMW/NEI2i1hNvE=";
          };
        };
        py-hpp-fcl = pkgs.python3Packages.toPythonModule (hpp-fcl.override { pythonSupport = true; });
        pinocchio = pkgs.pinocchio.override { inherit hpp-fcl; };
        py-pinocchio = pkgs.python3Packages.pinocchio.override {
          inherit hpp-fcl;
          python3Packages = {
            inherit (pkgs.python3Packages) boost eigenpy;
            hpp-fcl = py-hpp-fcl;
          };
        };
        py-example-robot-data = pkgs.python3Packages.example-robot-data.override {
          python3Packages = {
            pinocchio = py-pinocchio;
          };
        };
        crocoddyl = pkgs.crocoddyl.override { inherit pinocchio; };
        py-crocoddyl = pkgs.python3Packages.crocoddyl.override {
          inherit pinocchio;
          python3Packages = {
            inherit (pkgs.python3Packages) scipy;
            pinocchio = py-pinocchio;
            example-robot-data = py-example-robot-data;
          };
        };
      in
      {
        devShells.default = pkgs.mkShell {
          inputsFrom = [ py-crocoddyl ];
          packages = [ (pkgs.python3.withPackages (ps: [ py-crocoddyl ])) ];
        };
      }
    );
}
