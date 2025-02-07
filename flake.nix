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
              (_final: prev: {
                # Get pinocchio with #2566 until pinocchio > 3.3.1
                # to fix crocoddyl python imports on macos
                inherit (inputs.crocoddyl.packages.${system}) pinocchio;
                # Get croddyl with #1339
                # to have std::shared_ptr
                crocoddyl = prev.crocoddyl.overrideAttrs {
                  inherit (inputs.crocoddyl.packages.${system}.crocoddyl) src;
                };
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
