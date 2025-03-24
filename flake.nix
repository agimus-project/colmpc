{
  description = "Collision avoidance for MPC";

  inputs = {
    flake-parts.url = "github:hercules-ci/flake-parts";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

    # Remove once the next mim-solvers release reach nixpkgs/nixos-unstable
    mim-solvers = {
      # Move to master once https://github.com/machines-in-motion/mim_solvers/pull/48 is merged.
      url = "github:machines-in-motion/mim_solvers/main";
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
