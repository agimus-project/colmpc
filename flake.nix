{
  description = "Collision avoidance for MPC";

  inputs = {
    flake-parts.url = "github:hercules-ci/flake-parts";
    nixpkgs.url = "github:NixOS/nixpkgs/master";

    mim-solvers = {
      # url = "github:machines-in-motion/mim_solvers/devel";
      url = "github:MaximilienNaveau/mim_solvers/nix";
      inputs = {
        nixpkgs.follows = "nixpkgs";
        flake-parts.follows = "flake-parts";
      };
    };
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = inputs.nixpkgs.lib.systems.flakeExposed;
      perSystem =
        { pkgs, self', system, ... }:
        {
          _module.args.pkgs = import inputs.nixpkgs {
            inherit system;
            overlays = [
              (final: prev: {
                # patch mim-solvers for boost -> std shared_ptr
                mim-solvers = prev.mim-solvers.overrideAttrs (super: {
                  src = inputs.mim-solvers;
                  postPatch = ""; # disable default patch
                });
              })
            ];
          };
          apps.default = {
            type = "app";
            program = pkgs.python3.withPackages (_: [ self'.packages.default ]);
          };
          devShells.default = pkgs.mkShell { inputsFrom = [ self'.packages.default ]; };
          packages = {
            default = self'.packages.py-colmpc;
            colmpc = pkgs.callPackage ./. { };
            py-colmpc = pkgs.python3Packages.toPythonModule (
              self'.packages.colmpc.override { pythonSupport = true; }
            );
          };
        };
    };
}
