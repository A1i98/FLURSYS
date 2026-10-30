use serde_json::{json, Value};
use std::{
    io::{BufRead, BufReader, Write},
    process::{Command, Stdio},
    time::{Duration, Instant},
};

#[test]
fn stdio_mcp_controls_a_workspace_and_reports_background_failure() {
    let root = tempfile::tempdir().unwrap();
    let project = root.path().join("project");
    flursys::save_workspace(&project, &flursys::WorkbenchProject::blank("MCP test")).unwrap();
    let mut child = Command::new(env!("CARGO_BIN_EXE_flursys"))
        .args(["mcp", "--root", root.path().to_str().unwrap()])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()
        .unwrap();
    let mut input = child.stdin.take().unwrap();
    let mut output = BufReader::new(child.stdout.take().unwrap());
    fn send(input: &mut impl Write, output: &mut impl BufRead, request: Value) -> Value {
        writeln!(input, "{request}").unwrap();
        input.flush().unwrap();
        let mut line = String::new();
        output.read_line(&mut line).unwrap();
        serde_json::from_str(&line).unwrap()
    }
    fn call(input: &mut impl Write, output: &mut impl BufRead, name: &str, args: Value) -> Value {
        let r = send(
            input,
            output,
            json!({"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":name,"arguments":args}}),
        );
        let value: Value =
            serde_json::from_str(r["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
        json!({"error":r["result"]["isError"],"value":value})
    }
    let init = send(
        &mut input,
        &mut output,
        json!({"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-06-18","capabilities":{},"clientInfo":{"name":"test","version":"1"}}}),
    );
    assert_eq!(init["result"]["protocolVersion"], "2025-06-18");
    writeln!(
        input,
        "{}",
        json!({"jsonrpc":"2.0","method":"notifications/initialized"})
    )
    .unwrap();
    let list = send(
        &mut input,
        &mut output,
        json!({"jsonrpc":"2.0","id":2,"method":"tools/list"}),
    );
    assert!(list["result"]["tools"]
        .as_array()
        .unwrap()
        .iter()
        .any(|v| v["name"] == "project_solve"));
    let resources = send(
        &mut input,
        &mut output,
        json!({"jsonrpc":"2.0","id":4,"method":"resources/list"}),
    );
    assert_eq!(
        resources["result"]["resources"].as_array().unwrap().len(),
        2
    );
    let norms = call(
        &mut input,
        &mut output,
        "compare_samples",
        json!({"observed":[1.0,3.0],"expected":[1.0,2.0]}),
    );
    assert_eq!(norms["value"]["mean_absolute_error"], 0.5);
    let invalid_args = call(
        &mut input,
        &mut output,
        "project_solve",
        json!({"directory":"project","threads":"six"}),
    );
    assert_eq!(invalid_args["error"], true);
    let settings = call(
        &mut input,
        &mut output,
        "project_settings",
        json!({"directory":"project"}),
    );
    assert_eq!(settings["error"], false);
    let change = call(
        &mut input,
        &mut output,
        "project_update_solver",
        json!({"directory":"project","settings":{"threads":2}}),
    );
    assert_eq!(change["value"]["solver"]["threads"], 2);
    assert_eq!(flursys::load_workspace(&project).unwrap().solver.threads, 2);
    let invalid = call(
        &mut input,
        &mut output,
        "project_update_solver",
        json!({"directory":"project","settings":{"not_a_setting":3}}),
    );
    assert_eq!(invalid["error"], true);
    let escape = call(
        &mut input,
        &mut output,
        "project_inspect",
        json!({"directory":"/"}),
    );
    assert_eq!(escape["error"], true);
    let job = call(
        &mut input,
        &mut output,
        "project_solve",
        json!({"directory":"project","iterations":1,"threads":1}),
    );
    let id = job["value"]["job_id"].as_u64().unwrap();
    let deadline = Instant::now() + Duration::from_secs(30);
    loop {
        let status = call(&mut input, &mut output, "job_status", json!({"job_id":id}));
        if status["value"]["status"] != "running" {
            assert_eq!(status["value"]["status"], "failed");
            break;
        }
        assert!(Instant::now() < deadline);
        std::thread::sleep(Duration::from_millis(50));
    }
    drop(input);
    assert!(child.wait().unwrap().success());
}
