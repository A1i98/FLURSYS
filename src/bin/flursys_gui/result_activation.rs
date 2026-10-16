//! Connect completed and reopened workbench runs to the result viewer.
use super::*;

impl AppState {
    pub(super) fn install_result_dataset(&mut self, dataset: ResultDataset) -> Result<(), String> {
        let cache = ResultRenderCache::build(&dataset).map_err(|e| e.to_string())?;
        if dataset.mesh().dimension() == MeshDimension::TwoD {
            let (lo, hi) = cache.mesh_cache().bounds();
            self.mesh_view.fit(Some((lo.x, lo.y, hi.x, hi.y)));
        }
        self.post.reset_result();
        self.active_result = Some(dataset);
        self.result_render_cache = Some(cache);
        self.active_result_probe = None;
        self.streamline_paths.clear();
        self.streamline_seed_mode = false;
        self.result_manual_range = None;
        Ok(())
    }

    pub(super) fn activate_current_solution(&mut self, id: &str) -> Result<(), String> {
        let mesh = self
            .workbench
            .mesh()
            .ok_or("No completed mesh is available")?;
        let solution = self
            .workbench
            .solution()
            .ok_or("No completed solution is available")?;
        let dataset = ResultDataset::from_values(
            id,
            &mesh.mesh,
            solution.pressure.values().to_vec(),
            solution.velocity.values().to_vec(),
        )
        .map_err(|e| e.to_string())?;
        self.wb_residual_history = solution
            .report
            .continuity_history
            .iter()
            .rev()
            .take(400)
            .rev()
            .filter(|v| v.is_finite())
            .map(|v| ResidualSample {
                continuity: *v,
                momentum: 0.0,
                pressure: 0.0,
            })
            .collect();
        let histories = [
            solution.report.simple.momentum_relative_history.clone(),
            solution.report.simple.velocity_change_history.clone(),
            solution.report.simple.pressure_linear_history.clone(),
        ];
        self.install_result_dataset(dataset)?;
        self.post.metric_histories = histories;
        if let Some((mesh,solution))=self.workbench.mesh().zip(self.workbench.solution()) {
            let reports=flursys::workbench::boundary_reports::boundary_flow_reports(&mesh.mesh,solution,self.workbench.material().density);
            self.post.boundary_summary=result_analysis::boundary_summary(&reports);
            self.post.boundary_reports=reports;
        }
        Ok(())
    }

    pub(super) fn ensure_result_active(&mut self) -> Result<(), String> {
        if self.active_result.is_some() {
            return Ok(());
        }
        if self.workbench.solution().is_some() {
            return self.activate_current_solution("current-solve");
        }
        let id = self
            .workbench_project
            .runs
            .iter()
            .rev()
            .find(|r| r.solution_path.is_some())
            .map(|r| r.id.clone());
        if let Some(id) = id {
            self.activate_persisted_result(&id)
        } else {
            Err("No completed result is available. Run the workbench solver first.".into())
        }
    }
}

pub(super) fn refresh(ui: &MainWindow, state: &AppState) {
    if let Some(result) = state.active_result() {
        let record = state
            .workbench_project
            .runs
            .iter()
            .find(|r| r.id == result.run_id());
        let summary = record
            .map(|r| {
                format!(
                    "Run {} · {}\nIterations {}\nFinal continuity {}",
                    r.id,
                    run_status_label(&r.status),
                    r.iterations.map_or("—".into(), |i| i.to_string()),
                    r.continuity_residual
                        .map_or("—".into(), |v| format!("{v:.3e}"))
                )
            })
            .unwrap_or_else(|| solution_summary_text(&state.workbench));
        ui.set_residual_summary(summary.into());
        ui.set_field_summary(
            format!(
                "{} cells · {} nodes\nSpeed range {}\nPressure range {}",
                result.mesh().cell_count(),
                result.mesh().points().len(),
                range(result, ResultFieldKind::VelocityMagnitude),
                range(result, ResultFieldKind::Pressure)
            )
            .into(),
        );
        ui.set_force_summary("Force coefficients are not evaluated for this result.".into());
    } else if state.workbench.geometry().vertices().next().is_some() {
        ui.set_residual_summary(solution_summary_text(&state.workbench).into());
        ui.set_field_summary(
            "No active result dataset. Select Results after completing a solve.".into(),
        );
        ui.set_force_summary("Force coefficients are not evaluated for this run.".into());
    }
    let values = state
        .wb_residual_history
        .iter()
        .map(|s| s.continuity)
        .filter(|v| v.is_finite() && *v > 0.0)
        .collect::<Vec<_>>();
    ui.set_continuity_chart_caption(if values.is_empty() {
        "No continuity history available for this run.".into()
    } else {
        let lo = values.iter().copied().fold(f64::INFINITY, f64::min);
        let hi = values.iter().copied().fold(0.0, f64::max);
        format!(
            "Logarithmic scale · {lo:.3e} … {hi:.3e} · last {} samples",
            state.wb_residual_history.len()
        )
        .into()
    });
}
fn range(result: &ResultDataset, field: ResultFieldKind) -> String {
    result
        .scalar_range(field)
        .map(|(a, b)| format!("{a:.4e} … {b:.4e}"))
        .unwrap_or_else(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use slint::platform::{
        software_renderer::{MinimalSoftwareWindow, RepaintBufferType},
        Platform, WindowAdapter,
    };
    struct SoftwarePlatform(Rc<MinimalSoftwareWindow>);
    impl Platform for SoftwarePlatform {
        fn create_window_adapter(&self) -> Result<Rc<dyn WindowAdapter>, slint::PlatformError> {
            Ok(self.0.clone())
        }
        fn duration_since_start(&self) -> Duration {
            Duration::from_secs(1)
        }
    }
    #[test]
    fn results_navigation_loads_saved_fields_without_selecting_a_run_or_solving() {
        let window = MinimalSoftwareWindow::new(RepaintBufferType::NewBuffer);
        slint::platform::set_platform(Box::new(SoftwarePlatform(window.clone()))).unwrap();
        let ui = MainWindow::new().unwrap();
        let temporary = tempfile::tempdir().unwrap();
        let workspace = std::env::var_os("FLURSYS_VERIFY_WORKSPACE")
            .map(PathBuf::from)
            .unwrap_or_else(|| temporary.path().to_path_buf());
        let mut state = AppState::new();
        if workspace == temporary.path() {
            let mut run = state
                .workbench_project
                .next_run(flursys::RunStatus::Converged);
            run.iterations = Some(7);
            run.continuity_residual = Some(1e-12);
            run.solution_path = Some("runs/run-0001/solution.vtk".into());
            std::fs::create_dir_all(workspace.join("runs/run-0001")).unwrap();
            std::fs::write(workspace.join(run.solution_path.as_ref().unwrap()),"# vtk DataFile Version 3.0\nrun\nASCII\nDATASET UNSTRUCTURED_GRID\nPOINTS 3 double\n0 0 0\n1 0 0\n0 1 0\nCELLS 1 4\n3 0 1 2\nCELL_TYPES 1\n7\nCELL_DATA 1\nSCALARS pressure double 1\nLOOKUP_TABLE default\n2\nSCALARS velocity_magnitude double 1\nLOOKUP_TABLE default\n5\nVECTORS velocity double\n3 4 0\n").unwrap();
            std::fs::write(
                workspace.join(&run.report_path),
                r#"{"continuity_history":[1e-11,3e-12,1e-12]}"#,
            )
            .unwrap();
            state.workbench_project.runs.push(run);
        } else {
            state.workbench_project = flursys::load_workspace(&workspace).unwrap();
        }
        state.workspace_path = Some(workspace);
        let state = Rc::new(RefCell::new(state));
        bind_callbacks(&ui, &state);
        apply_workflow_step(&ui, &mut state.borrow_mut(), 4);
        let cells = state.borrow().active_result().unwrap().mesh().cell_count();
        assert!(cells > 0);
        assert!(
            state.borrow().workbench.solution().is_none(),
            "Navigation must use the saved artifact without running a solver"
        );
        assert!(!ui.get_animation_status().contains("Frame 0"));
        assert!(!ui.get_residual_summary().contains("Waiting"));
        assert!(ui.get_wb_solution_summary().contains("Run run-0001"));
        assert!(ui.get_field_summary().contains(&format!("{cells} cells")));
        let image = ui.get_visualization_image().to_rgba8().unwrap();
        let first = image.as_bytes()[..4].to_vec();
        assert!(
            image
                .as_bytes()
                .chunks_exact(4)
                .filter(|pixel| *pixel != first.as_slice())
                .count()
                > 100
        );
        ui.show().unwrap();
        for field in [0, 1] {
            ui.invoke_select_result_field(field);
            if field == 1 {
                assert!(ui.get_visualization_title().contains("PRESSURE"));
            }
            window.set_size(slint::PhysicalSize::new(1440, 900));
            window.request_redraw();
            let mut pixels = vec![slint::Rgb8Pixel::default(); 1440 * 900];
            window.draw_if_needed(|renderer| {
                renderer.render(&mut pixels, 1440);
            });
            let mut ppm = b"P6\n1440 900\n255\n".to_vec();
            for p in pixels {
                ppm.extend([p.r, p.g, p.b]);
            }
            std::fs::write(format!("/tmp/flursys-results-verified-{field}.ppm"), ppm).unwrap();
        }
        eprintln!(
            "Saved result displayed without solving: {cells} cells; {}",
            ui.get_residual_summary()
        );
    }

    #[test]
    #[ignore = "requires Gmsh"]
    fn worker_completion_activates_saved_fields_automatically() {
        let directory = tempfile::tempdir().unwrap();
        let mut state = AppState::new();
        let r = state.workbench.add_rectangle(1.0, 1.0).unwrap();
        let edge = GeometrySelectionTarget::Edge;
        state
            .workbench
            .set_boundary_group("top", vec![edge(r.top)], false)
            .unwrap();
        state
            .workbench
            .set_boundary_group(
                "walls",
                vec![edge(r.left), edge(r.right), edge(r.bottom)],
                false,
            )
            .unwrap();
        state
            .workbench
            .assign_boundary(
                "top",
                IncompressibleBoundaryCondition::MovingWall {
                    velocity: Vec3::new(1.0, 0.0, 0.0),
                },
            )
            .unwrap();
        state
            .workbench
            .assign_boundary("walls", IncompressibleBoundaryCondition::NoSlipWall)
            .unwrap();
        state
            .workbench
            .set_mesh_configuration(MeshDimension::TwoD, 0.15, 0.075, 0.15, 1)
            .unwrap();
        let (export, options) = state.workbench.mesh_generation_inputs().unwrap();
        let mesh = state
            .workbench
            .mesher()
            .generate(&export.document, &options)
            .unwrap();
        state.workbench.install_mesh(mesh);
        state.workspace_path = Some(directory.path().to_path_buf());
        state.solving = true;
        state
            .workbench_jobs
            .start_solve(state.workbench.prepare_case().unwrap())
            .unwrap();
        let deadline = std::time::Instant::now() + Duration::from_secs(20);
        while state.solving {
            state.drain_workbench_jobs();
            assert!(
                std::time::Instant::now() < deadline,
                "Solver worker timed out"
            );
            std::thread::sleep(Duration::from_millis(2));
        }
        assert!(state.workbench.solution().is_some());
        let solution = state.workbench.solution().unwrap();
        assert_eq!(
            solution.report.simple.momentum_relative_history.len(),
            solution.report.outer_iterations
        );
        assert_eq!(
            solution.report.simple.momentum_relative_history.last(),
            Some(&solution.report.simple.final_momentum_relative_residual)
        );
        assert_eq!(
            solution.report.simple.velocity_change_history.last(),
            Some(&solution.report.simple.final_velocity_relative_change)
        );
        assert!(directory
            .path()
            .join("runs/run-0001/solution.vtk")
            .is_file());
        assert_eq!(state.active_result().unwrap().run_id(), "run-0001");
        assert_eq!(
            state.active_result().unwrap().pressure().len(),
            state.workbench.mesh().unwrap().mesh.cell_count()
        );
        assert!(state.wb_residual_history.len() > 1);
        state.active_result = None;
        state.result_render_cache = None;
        state.activate_persisted_result("run-0001").unwrap();
        assert!(
            state.wb_residual_history.len() > 1,
            "Saved reports must restore the convergence history"
        );
        assert!(state.post.metric_histories.iter().all(|v| !v.is_empty()));
    }

    #[test]
    fn workbench_chart_resolves_continuity_below_one_e_minus_ten() {
        let history = (0..20)
            .map(|i| ResidualSample {
                continuity: 10f64.powf(-12.0 - i as f64 / 20.0),
                momentum: 0.0,
                pressure: 0.0,
            })
            .collect();
        let image = render::render_workbench_continuity_chart(&history)
            .to_rgba8()
            .unwrap();
        let rows = image
            .as_bytes()
            .chunks_exact(PREVIEW_WIDTH as usize * 4)
            .filter(|row| row.chunks_exact(4).any(|p| p == [77, 168, 184, 255]))
            .count();
        assert!(
            rows > 50,
            "Small residuals must span the graph rather than collapse onto the border: {rows} rows"
        );
    }
}
