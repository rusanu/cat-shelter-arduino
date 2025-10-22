## Project ##
This project is a AI thinker ESP32 CAM project using PlatformIO and Arduino framework
Using C++ for logic, agent generated code should be C++
The project purpose is to control a winter cat shelter
- control the relay to activate a heated blanket when cold outside and cat present
- take a picture and upload to an S3 bucket every 60 minutes or whenever PIR sensor detects presence, but not more than once every 5 min
- turn off WIFI when idle

## Engineering ##
This is a hobby project with only one deployment. Do not overengineer. Is more im portant to keep the code easy to udnerstand and maintain.
All testing is manual. Do not plan to testing or acceptance, do not write tests. Just ask the user to verify the changes.
Always confirm with the user that the issue you work on is completed and verified.
When resolving an issue, commit. Never push because push may trigger Github Actions and consume free project actions time quota. the user will push manually when needed.

### Hardware ###
There are 3 devices connected
- GPIO12 connects to a relay control (output). When HIGH it will activate a termal blanket
- GPIO13 is connected to a PIR sensor (input). It detecs cat(s) presence
- GPIO14 is connected to a DHT22/AM2302 temperature/humidity senzor.
The onboard camera and wifi are functional and used by the project. The SD card is not enabled.


## Coding ##
- do not attempt to upload the firmware yourself. The hardware requires me to change wiring to put it in boot mode before upload
- **IMPORTANT** do not store secrets and credentials in code. The repo is public. Use indirection methods like .h file excluded from git, environment variables, .env files etc
- if you need to generate temporary or debug files, do not place them in the project folder. Use %TMP%


## Issue Tracking with bd (beads)

**IMPORTANT**: This project uses **bd (beads)** for ALL issue tracking. Do NOT use markdown TODOs, task lists, or other tracking methods.

### Why bd?

- Dependency-aware: Track blockers and relationships between issues
- Git-friendly: Auto-syncs to JSONL for version control
- Agent-optimized: JSON output, ready work detection, discovered-from links
- Prevents duplicate tracking systems and confusion

### Quick Start

**Check for ready work:**
```bash
bd ready --json
```

**Create new issues:**
```bash
bd create "Issue title" -t bug|feature|task -p 0-4 --json
bd create "Issue title" -p 1 --deps discovered-from:bd-123 --json
```

**Claim and update:**
```bash
bd update bd-42 --status in_progress --json
bd update bd-42 --priority 1 --json
```

**Complete work:**
```bash
bd close bd-42 --reason "Completed" --json
```

### Issue Types

- `bug` - Something broken
- `feature` - New functionality
- `task` - Work item (tests, docs, refactoring)
- `epic` - Large feature with subtasks
- `chore` - Maintenance (dependencies, tooling)

### Priorities

- `0` - Critical (security, data loss, broken builds)
- `1` - High (major features, important bugs)
- `2` - Medium (default, nice-to-have)
- `3` - Low (polish, optimization)
- `4` - Backlog (future ideas)

### Workflow for AI Agents

1. **Check ready work**: `bd ready` shows unblocked issues
2. **Claim your task**: `bd update <id> --status in_progress`
3. **Work on it**: Implement, test, document
4. **Discover new work?** Create linked issue:
   - `bd create "Found bug" -p 1 --deps discovered-from:<parent-id>`
5. **Complete**: `bd close <id> --reason "Done"`

### Auto-Sync

bd automatically syncs with git:
- Exports to `.beads/issues.jsonl` after changes (5s debounce)
- Imports from JSONL when newer (e.g., after `git pull`)
- No manual export/import needed!

### MCP Server (Recommended)

If using Claude or MCP-compatible clients, install the beads MCP server:

```bash
pip install beads-mcp
```

Add to MCP config (e.g., `~/.config/claude/config.json`):
```json
{
  "beads": {
    "command": "beads-mcp",
    "args": []
  }
}
```

Then use `mcp__beads__*` functions instead of CLI commands.

### Important Rules

- ✅ Use bd for ALL task tracking
- ✅ Always use `--json` flag for programmatic use
- ✅ Link discovered work with `discovered-from` dependencies
- ✅ Check `bd ready` before asking "what should I work on?"
- ❌ Do NOT create markdown TODO lists
- ❌ Do NOT use external issue trackers
- ❌ Do NOT duplicate tracking systems

For more details, see README.md and QUICKSTART.md.
