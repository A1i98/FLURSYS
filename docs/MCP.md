# MCP

FLURSYS provides a local MCP server over stdio.

```sh
cargo build --release --locked --no-default-features --bin flursys
```

Set the executable path and project root for your system. The root limits which files the server can access.

## Codex and Claude Code

```sh
codex mcp add flursys -- /path/to/flursys mcp --root /path/to/projects
claude mcp add --transport stdio --scope user flursys -- /path/to/flursys mcp --root /path/to/projects
```

## Other clients

Add the following to your client's MCP configuration:

```json
{
  "mcpServers": {
    "flursys": {
      "command": "/path/to/flursys",
      "args": ["mcp", "--root", "/path/to/projects"]
    }
  }
}
```

- Cursor: `.cursor/mcp.json` or `~/.cursor/mcp.json`
- Gemini CLI: `.gemini/settings.json` or `~/.gemini/settings.json`
- Claude Desktop: `claude_desktop_config.json`
- Copilot CLI: `.mcp.json` or `~/.copilot/mcp-config.json`
- Cascade: its MCP configuration file

For VS Code, use `.vscode/mcp.json` with `servers` instead of `mcpServers` and add `"type": "stdio"` to the server entry. On Windows, use the `.exe` path and escape backslashes in JSON.

## Usage

Ask your agent to inspect a project, configure and run a case, or review saved results. The client discovers available operations from the server. Long-running jobs return an ID; keep the connection open while they run. Gmsh is required for mesh generation.
