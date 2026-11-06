## Terminal Execution Rules
- When running Git commands or inspecting files in specific folders, do not rely on passing the `cd` tool parameter.
- Instead, specify the working directory directly in the command string:
  - For Git: Use `git -C <dir> <subcommand>` (e.g., `git -C path/to/repo log --oneline`).
  - For other commands: Chain `cd` inline (e.g., `cd /path/to/dir && command`).
