{
  description = "Collision avoidance for MPC";

  inputs = {
    flake-parts.url = "github:hercules-ci/flake-parts";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

    # override for boost -> std shared_ptr
    crocoddyl = {
      url = "github:loco-3d/crocoddyl/release/3.0.0";
      inputs.flake-parts.follows = "flake-parts";
      inputs.nixpkgs.follows = "nixpkgs";
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
                # Get pinocchio with #2566 until pinocchio > 3.3.1
                # to fix crocoddyl python imports on macos
                # And croddyl with #1339
                # to have std::shared_ptr
                inherit (inputs.crocoddyl.packages.${system}) crocoddyl pinocchio;
                # patch mim-solvers for boost -> std shared_ptr
                mim-solvers = prev.mim-solvers.overrideAttrs (super: {
                  patches = (super.patches or []) ++ [
                    (final.fetchpatch {
                      url = "https://github.com/machines-in-motion/mim_solvers/pull/44.patch";
                      hash = "sha256-2LmNM6Hg5WwY8KlZzwxGO3VvozSHhnnKrS4vfGXzvtE=";
                    })
                  ];
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
