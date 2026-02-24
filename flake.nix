{
  description = "Collision avoidance for MPC";

  inputs = {
    gazebros2nix.url = "github:gepetto/gazebros2nix";
    flake-parts.follows = "gazebros2nix/flake-parts";
    nixpkgs.follows = "gazebros2nix/nixpkgs";
    nix-ros-overlay.follows = "gazebros2nix/nix-ros-overlay";
    systems.follows = "gazebros2nix/systems";
    treefmt-nix.follows = "gazebros2nix/treefmt-nix";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, self, ... }:
      {
        systems = [ "x86_64-linux" ];
        imports = [
          inputs.gazebros2nix.flakeModule
          {
            gazebros2nix = {
              packages = {
                colmpc = _final: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./examples
                      ./include
                      ./python
                      ./tests
                      ./CMakeLists.txt
                      ./package.xml
                      ./pyproject.toml
                    ];
                  };
                };
              };
            };
          }
        ];
        perSystem =
          { pkgs, ... }:
          {
            packages.default = pkgs.python3Packages.colmpc;
          };
      }
    );
}
