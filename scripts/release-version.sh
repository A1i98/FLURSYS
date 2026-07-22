#!/usr/bin/env bash
# Create a verified FLURSYS release commit and annotated SemVer tag.
#
# The GitHub release workflow publishes binaries after the tag is pushed.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MANIFEST="$REPO_ROOT/Cargo.toml"
LOCKFILE="$REPO_ROOT/Cargo.lock"

usage() {
    cat <<'EOF'
Usage: scripts/release-version.sh <major|minor|patch|release|alpha|beta|rc|prepatch|preminor|premajor|prerelease|X.Y.Z[-channel.N]> [options]

Options:
  --push                 Push the release commit and vX.Y.Z tag to origin.
  --dry-run              Print the calculated version without changing files.
  --allow-dirty          Allow pre-existing worktree changes (normally refused).
  --channel <channel>    Prerelease channel: alpha, beta, or rc (default: alpha).
  --base <bump>          Base bump for alpha/beta/rc from a stable version:
                          patch, minor, or major (default: patch).
  --message <message>    Override the release commit message.
  -h, --help             Show this help text.

Examples:
  scripts/release-version.sh patch --dry-run
  scripts/release-version.sh minor --push
  scripts/release-version.sh alpha --base minor --push
  scripts/release-version.sh prerelease --channel beta --push
  scripts/release-version.sh premajor --channel rc --push
  scripts/release-version.sh release --push
  scripts/release-version.sh 1.2.0 --message "chore(release): v1.2.0" --push
EOF
}

fail() {
    printf 'release-version failed: %s\n' "$*" >&2
    exit 1
}

run() {
    printf '\n==> %s\n' "$*"
    "$@"
}

is_semver() {
    [[ "$1" =~ ^[0-9]+\.[0-9]+\.[0-9]+(-[0-9A-Za-z.-]+)?$ ]]
}

current_version() {
    awk '
        $0 == "[package]" { in_package = 1; next }
        /^\[/ { in_package = 0 }
        in_package && $1 == "version" {
            value = $3
            gsub(/"/, "", value)
            print value
            exit
        }
    ' "$MANIFEST"
}

parse_version() {
    local version="$1"
    if [[ "$version" =~ ^([0-9]+)\.([0-9]+)\.([0-9]+)(-([0-9A-Za-z.-]+))?$ ]]; then
        VERSION_MAJOR="${BASH_REMATCH[1]}"
        VERSION_MINOR="${BASH_REMATCH[2]}"
        VERSION_PATCH="${BASH_REMATCH[3]}"
        VERSION_PRERELEASE="${BASH_REMATCH[5]:-}"
        VERSION_CORE="${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}"
        return 0
    fi
    return 1
}

next_stable_version() {
    local current="$1"
    local bump="$2"
    parse_version "$current" || fail "cannot parse current version '$current'"
    case "$bump" in
        patch) printf '%s.%s.%s\n' "$VERSION_MAJOR" "$VERSION_MINOR" "$((VERSION_PATCH + 1))" ;;
        minor) printf '%s.%s.0\n' "$VERSION_MAJOR" "$((VERSION_MINOR + 1))" ;;
        major) printf '%s.0.0\n' "$((VERSION_MAJOR + 1))" ;;
        *) fail "invalid stable bump '$bump'" ;;
    esac
}

next_channel_version() {
    local current="$1"
    local channel="$2"
    local base_bump="$3"
    local next_core current_channel current_number
    parse_version "$current" || fail "cannot parse current version '$current'"

    if [[ -z "$VERSION_PRERELEASE" ]]; then
        next_core="$(next_stable_version "$current" "$base_bump")"
        printf '%s-%s.1\n' "$next_core" "$channel"
        return
    fi

    if [[ "$VERSION_PRERELEASE" =~ ^(alpha|beta|rc)\.([0-9]+)$ ]]; then
        current_channel="${BASH_REMATCH[1]}"
        current_number="${BASH_REMATCH[2]}"
        if [[ "$current_channel" == "$channel" ]]; then
            printf '%s-%s.%s\n' "$VERSION_CORE" "$channel" "$((current_number + 1))"
        else
            printf '%s-%s.1\n' "$VERSION_CORE" "$channel"
        fi
        return
    fi

    fail "current prerelease '$VERSION_PRERELEASE' is not an alpha, beta, or rc version; use an explicit version"
}

next_version() {
    local current="$1"
    local bump="$2"
    local channel="$3"
    local base_bump="$4"
    local next_core

    if is_semver "$bump"; then
        printf '%s\n' "$bump"
        return
    fi

    case "$bump" in
        major|minor|patch) next_stable_version "$current" "$bump" ;;
        alpha|beta|rc) next_channel_version "$current" "$bump" "$base_bump" ;;
        prerelease) next_channel_version "$current" "$channel" "$base_bump" ;;
        prepatch|preminor|premajor)
            next_core="$(next_stable_version "$current" "${bump#pre}")"
            printf '%s-%s.1\n' "$next_core" "$channel"
            ;;
        release)
            parse_version "$current" || fail "cannot parse current version '$current'"
            [[ -n "$VERSION_PRERELEASE" ]] || fail "release requires a current alpha, beta, or rc version"
            printf '%s\n' "$VERSION_CORE"
            ;;
        *)
            fail "unsupported bump '$bump'"
            ;;
    esac
}

replace_manifest_version() {
    local version="$1"
    local temporary
    temporary="$(mktemp "$REPO_ROOT/.release-version.XXXXXX")"
    awk -v next_version="$version" '
        $0 == "[package]" { in_package = 1 }
        /^\[/ && $0 != "[package]" { in_package = 0 }
        in_package && !replaced && $1 == "version" {
            print "version = \"" next_version "\""
            replaced = 1
            next
        }
        { print }
        END {
            if (!replaced) exit 2
        }
    ' "$MANIFEST" >"$temporary" || {
        rm -f "$temporary"
        fail "could not update the package version in Cargo.toml"
    }
    mv "$temporary" "$MANIFEST"
}

ensure_clean_worktree() {
    local status
    status="$(git status --porcelain)"
    [[ -z "$status" ]] || fail "working tree is not clean; commit/stash changes first or use --allow-dirty"
}

BUMP=""
PUSH=false
DRY_RUN=false
ALLOW_DIRTY=false
COMMIT_MESSAGE=""
CHANNEL="alpha"
BASE_BUMP="patch"

while [[ $# -gt 0 ]]; do
    case "$1" in
        major|minor|patch|release|alpha|beta|rc|prepatch|preminor|premajor|prerelease)
            [[ -z "$BUMP" ]] || fail "only one version bump can be supplied"
            BUMP="$1"
            ;;
        --push) PUSH=true ;;
        --dry-run) DRY_RUN=true ;;
        --allow-dirty) ALLOW_DIRTY=true ;;
        --channel)
            [[ $# -ge 2 ]] || fail "--channel requires alpha, beta, or rc"
            CHANNEL="$2"
            shift
            ;;
        --base)
            [[ $# -ge 2 ]] || fail "--base requires patch, minor, or major"
            BASE_BUMP="$2"
            shift
            ;;
        --message)
            [[ $# -ge 2 ]] || fail "--message requires a value"
            COMMIT_MESSAGE="$2"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            if [[ -z "$BUMP" ]] && is_semver "$1"; then
                BUMP="$1"
            else
                fail "unknown argument '$1'"
            fi
            ;;
    esac
    shift
done

[[ -n "$BUMP" ]] || {
    usage >&2
    exit 1
}
[[ "$CHANNEL" =~ ^(alpha|beta|rc)$ ]] || fail "--channel must be alpha, beta, or rc"
[[ "$BASE_BUMP" =~ ^(patch|minor|major)$ ]] || fail "--base must be patch, minor, or major"

cd "$REPO_ROOT"
git rev-parse --is-inside-work-tree >/dev/null 2>&1 || fail "not inside a git repository"
[[ -f "$MANIFEST" ]] || fail "Cargo.toml is missing"

if ! "$ALLOW_DIRTY"; then
    ensure_clean_worktree
fi

CURRENT_VERSION="$(current_version)"
is_semver "$CURRENT_VERSION" || fail "Cargo.toml has an invalid package version: '$CURRENT_VERSION'"
NEXT_VERSION="$(next_version "$CURRENT_VERSION" "$BUMP" "$CHANNEL" "$BASE_BUMP")"
[[ "$NEXT_VERSION" != "$CURRENT_VERSION" ]] || fail "target version equals the current version"
TAG="v$NEXT_VERSION"
if [[ "$NEXT_VERSION" == *-* ]]; then
    RELEASE_CHANNEL="${NEXT_VERSION#*-}"
    RELEASE_CHANNEL="${RELEASE_CHANNEL%%.*}"
else
    RELEASE_CHANNEL="stable"
fi

git rev-parse -q --verify "refs/tags/$TAG" >/dev/null 2>&1 && fail "tag already exists: $TAG"

if "$DRY_RUN"; then
    printf '%s\n' \
        "release-version dry-run" \
        "Current version: $CURRENT_VERSION" \
        "Requested bump: $BUMP" \
        "Release channel: $RELEASE_CHANNEL" \
        "Prerelease base bump policy: $BASE_BUMP" \
        "Target version: $NEXT_VERSION" \
        "Tag: $TAG" \
        "Push after tagging: $PUSH" \
        "No files were changed."
    exit 0
fi

MANIFEST_BACKUP="$(mktemp)"
LOCKFILE_BACKUP="$(mktemp)"
cp "$MANIFEST" "$MANIFEST_BACKUP"
if [[ -f "$LOCKFILE" ]]; then
    cp "$LOCKFILE" "$LOCKFILE_BACKUP"
fi
VERSION_CHANGED=false
COMMITTED=false

cleanup() {
    local exit_code=$?
    if [[ $exit_code -ne 0 && "$VERSION_CHANGED" == true && "$COMMITTED" == false ]]; then
        cp "$MANIFEST_BACKUP" "$MANIFEST"
        if [[ -s "$LOCKFILE_BACKUP" ]]; then
            cp "$LOCKFILE_BACKUP" "$LOCKFILE"
        fi
        printf 'Restored Cargo.toml and Cargo.lock after the failed release check.\n' >&2
    fi
    rm -f "$MANIFEST_BACKUP" "$LOCKFILE_BACKUP"
}
trap cleanup EXIT

replace_manifest_version "$NEXT_VERSION"
VERSION_CHANGED=true

# Refresh the root-package entry in Cargo.lock before enforcing --locked.
run cargo check --all-targets --features gui
run cargo fmt --all -- --check
run cargo test --all-targets --features gui --locked
run cargo clippy --all-targets --features gui --locked -- -D warnings
run cargo build --release --features gui --locked --bin flursys --bin flursys-gui

FILES_TO_COMMIT=(Cargo.toml)
[[ -f "$LOCKFILE" ]] && FILES_TO_COMMIT+=(Cargo.lock)
run git add "${FILES_TO_COMMIT[@]}"
git diff --cached --quiet && fail "version bump produced no staged changes"

if [[ -z "$COMMIT_MESSAGE" ]]; then
    COMMIT_MESSAGE="chore(release): v$NEXT_VERSION"
fi
run git commit -m "$COMMIT_MESSAGE"
COMMITTED=true
run git tag -a "$TAG" -m "Release $TAG"

if "$PUSH"; then
    BRANCH="$(git branch --show-current)"
    [[ -n "$BRANCH" ]] || fail "cannot push from a detached HEAD"
    run git push origin "$BRANCH"
    run git push origin "$TAG"
fi

printf '%s\n' \
    "Release commit created: $COMMIT_MESSAGE" \
    "Tag created: $TAG" \
    "$($PUSH && printf 'Pushed commit and tag to origin.' || printf 'Not pushed; rerun with --push when ready.')"
