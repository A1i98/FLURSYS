use super::project::{CaseTemplateManifest, MeshRecipe, WorkbenchMeshSettings, WorkbenchProject};
use crate::{unstructured_incompressible::TimeIntegration, Vec3};
use serde::{Deserialize, Serialize};
use std::path::{Component, Path, PathBuf};

pub const CASE_FORMAT_VERSION: u32 = 2;
fn revision() -> String {
    "1".into()
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct CaseMetadata {
    #[serde(default = "revision")]
    pub revision: String,
    pub sources: Vec<Source>,
    pub dimensionless: Vec<Dimensionless>,
    pub display: DisplayDefaults,
    pub comparisons: Vec<Comparison>,
    pub verification: Verification,
    pub presets: Vec<Preset>,
    pub assets: Vec<Asset>,
}
impl Default for CaseMetadata {
    fn default() -> Self {
        Self {
            revision: revision(),
            sources: vec![],
            dimensionless: vec![],
            display: Default::default(),
            comparisons: vec![],
            verification: Verification {
                status: "unvalidated".into(),
                ..Default::default()
            },
            presets: vec![],
            assets: vec![],
        }
    }
}
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Source {
    pub id: String,
    pub title: String,
    pub authors: Vec<String>,
    pub year: Option<u32>,
    pub doi: Option<String>,
    pub url: Option<String>,
    pub relationship: String,
    pub notes: String,
}
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Scale {
    pub symbol: String,
    pub value: f64,
    pub unit: String,
    pub definition: String,
}
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Dimensionless {
    pub symbol: String,
    pub value: Option<f64>,
    pub formula: String,
    pub scales: Vec<Scale>,
    pub notes: String,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct DisplayDefaults {
    pub field: String,
    pub mode: String,
    pub palette: String,
    pub plane: String,
    pub vectors: bool,
    pub cell_edges: bool,
    pub levels: usize,
    pub probes: Vec<Probe>,
    pub section: Option<super::plane_monitor::FlowPlaneMonitor>,
    pub plots: Vec<Plot>,
}
impl Default for DisplayDefaults {
    fn default() -> Self {
        Self {
            field: "speed".into(),
            mode: "filled-contours".into(),
            palette: "viridis".into(),
            plane: "surface".into(),
            vectors: true,
            cell_edges: false,
            levels: 12,
            probes: vec![],
            section: None,
            plots: vec![],
        }
    }
}
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Probe {
    pub name: String,
    pub position: Vec3,
    pub quantities: Vec<String>,
    pub sampling: String,
}
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Plot {
    pub name: String,
    pub kind: String,
    pub quantity: String,
    pub start: Option<Vec3>,
    pub end: Option<Vec3>,
    pub notes: String,
}
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Verification {
    pub status: String,
    pub evidence: Vec<String>,
    pub notes: String,
}
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Comparison {
    pub id: String,
    pub quantity: String,
    pub unit: String,
    pub source_id: Option<String>,
    pub kind: String,
    pub asset: Option<PathBuf>,
    pub expression: Option<String>,
    pub target: Option<f64>,
    pub time: Option<f64>,
    pub coordinate_column: Option<String>,
    pub value_column: Option<String>,
    pub error_metric: String,
    pub absolute_tolerance: Option<f64>,
    pub relative_tolerance: Option<f64>,
    pub procedure: String,
}
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Preset {
    pub id: String,
    pub name: String,
    pub description: String,
    pub mesh: Option<WorkbenchMeshSettings>,
    pub mesh_recipe: Option<MeshRecipe>,
    pub time: Option<TimeIntegration>,
    pub physical_end_time: Option<f64>,
    pub compressible_time: Option<CompressibleTime>,
}
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct CompressibleTime {
    pub courant: f64,
    pub max_time_step: f64,
    pub end_time: f64,
    pub max_steps: usize,
}
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Asset {
    pub path: PathBuf,
    pub role: String,
    pub media_type: String,
    pub sha256: Option<String>,
    pub linked_script: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub content: Option<String>,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CaseContext {
    pub id: String,
    pub revision: String,
    pub selected_preset: Option<String>,
    pub metadata: CaseMetadata,
}

pub fn relative_asset(path: &Path) -> Result<(), String> {
    if path.as_os_str().is_empty()
        || path
            .components()
            .any(|c| !matches!(c, Component::Normal(_)))
        || (path.to_string_lossy().contains('\\') || path.to_string_lossy().contains(':'))
    {
        return Err(format!(
            "Case asset path must be a contained relative path: {}",
            path.display()
        ));
    }
    Ok(())
}
fn digest(content: &str) -> String {
    use sha2::{Digest, Sha256};
    format!("{:x}", Sha256::digest(content.as_bytes()))
}
impl CaseMetadata {
    pub fn summary(&self) -> serde_json::Value {
        serde_json::json!({"revision":self.revision,"sources":self.sources,"dimensionless":self.dimensionless,"display":self.display,"comparisons":self.comparisons,"verification":self.verification,
            "presets":self.presets.iter().map(|p|serde_json::json!({"id":p.id,"name":p.name,"description":p.description})).collect::<Vec<_>>(),
            "assets":self.assets.iter().map(|a|serde_json::json!({"path":a.path,"role":a.role,"media_type":a.media_type,"sha256":a.sha256})).collect::<Vec<_>>()})
    }
    pub fn validate(&self) -> Result<(), String> {
        use std::collections::BTreeSet;
        let unique = |names: Vec<&str>, kind: &str| -> Result<(), String> {
            let mut seen = BTreeSet::new();
            for name in names {
                if name.trim().is_empty() || !seen.insert(name) {
                    return Err(format!("Empty or duplicate {kind}: {name}"));
                }
            }
            Ok(())
        };
        unique(
            self.sources.iter().map(|s| s.id.as_str()).collect(),
            "source ID",
        )?;
        unique(
            self.presets.iter().map(|p| p.id.as_str()).collect(),
            "preset ID",
        )?;
        unique(
            self.comparisons.iter().map(|p| p.id.as_str()).collect(),
            "comparison ID",
        )?;
        if let Some(section) = &self.display.section {
            section.validate()?;
        }
        if !(3..=32).contains(&self.display.levels) {
            return Err("Case contour levels must be 3–32".into());
        }
        if !["", "unvalidated", "inputs-reviewed", "numerically-verified"]
            .contains(&self.verification.status.as_str())
        {
            return Err("Unknown case verification status".into());
        }
        if self.verification.status == "numerically-verified"
            && self.verification.evidence.is_empty()
        {
            return Err("Numerically verified cases require evidence references".into());
        }
        for value in &self.dimensionless {
            if value.value.is_some_and(|v| !v.is_finite())
                || value.scales.iter().any(|s| !s.value.is_finite())
            {
                return Err("Non-finite dimensionless scale".into());
            }
        }
        let mut paths = BTreeSet::new();
        for asset in &self.assets {
            relative_asset(&asset.path)?;
            if !paths.insert(&asset.path) {
                return Err("Duplicate case asset path".into());
            }
            if let Some(hash) = &asset.sha256 {
                if hash.len() != 64 || !hash.bytes().all(|b| b.is_ascii_hexdigit()) {
                    return Err("Case SHA256 must contain 64 hexadecimal digits".into());
                }
            }
            if let (Some(hash), Some(content)) = (&asset.sha256, &asset.content) {
                if !digest(content).eq_ignore_ascii_case(hash) {
                    return Err(format!(
                        "Case asset checksum mismatch: {}",
                        asset.path.display()
                    ));
                }
            }
        }
        for item in &self.comparisons {
            if let Some(source) = &item.source_id {
                if !self.sources.iter().any(|s| &s.id == source) {
                    return Err(format!("Unknown comparison source: {source}"));
                }
            }
            if let Some(path) = &item.asset {
                if !self.assets.iter().any(|a| &a.path == path) {
                    return Err(format!("Missing reference asset: {}", path.display()));
                }
            }
            if [
                item.target,
                item.time,
                item.absolute_tolerance,
                item.relative_tolerance,
            ]
            .into_iter()
            .flatten()
            .any(|v| !v.is_finite())
            {
                return Err("Non-finite comparison parameter".into());
            }
            if item.absolute_tolerance.is_some_and(|v| v < 0.)
                || item.relative_tolerance.is_some_and(|v| v < 0.)
            {
                return Err("Comparison tolerances must be non-negative".into());
            }
        }
        for p in &self.display.probes {
            if ![p.position.x, p.position.y, p.position.z]
                .iter()
                .all(|v| v.is_finite())
            {
                return Err("Non-finite probe location".into());
            }
        }
        Ok(())
    }
}
pub fn hydrate(entry: &mut CaseTemplateManifest, root: &Path) -> Result<(), String> {
    for asset in &mut entry.metadata.assets {
        relative_asset(&asset.path)?;
        let file = root.join(&asset.path);
        if file.exists() {
            let real_root = std::fs::canonicalize(root).map_err(|e| e.to_string())?;
            let real = std::fs::canonicalize(&file).map_err(|e| e.to_string())?;
            if !real.starts_with(real_root) {
                return Err("Case asset symlink escapes its directory".into());
            }
            let content =
                std::fs::read_to_string(&real).map_err(|e| format!("{}: {e}", real.display()))?;
            if asset
                .sha256
                .as_ref()
                .is_some_and(|hash| !digest(&content).eq_ignore_ascii_case(hash))
            {
                return Err(format!("Case asset checksum mismatch: {}", file.display()));
            }
            asset.content = Some(content);
        }
        let content = asset
            .content
            .as_ref()
            .ok_or_else(|| format!("Missing portable asset: {}", asset.path.display()))?;
        if asset.role == "geometry" {
            let geometry: crate::GeometryTopology = serde_json::from_str(content)
                .map_err(|e| format!("Invalid case geometry asset: {e}"))?;
            if geometry != entry.project_template.geometry {
                entry.project_template.cad_document = None;
                entry.project_template.cad_geometry_hash = None;
            }
            entry.project_template.geometry = geometry;
        }
        if let Some(id) = &asset.linked_script {
            let script = entry
                .project_template
                .solver
                .scripting
                .scripts
                .get_mut(id)
                .ok_or_else(|| format!("Unknown linked script: {id}"))?;
            script.source = content.clone();
        }
    }
    entry.metadata.validate()
}
pub fn write_assets(root: &Path, assets: &[Asset]) -> Result<(), String> {
    use std::io::Write;
    for asset in assets {
        relative_asset(&asset.path)?;
        let content = asset
            .content
            .as_ref()
            .ok_or_else(|| format!("Asset has no portable content: {}", asset.path.display()))?;
        std::fs::create_dir_all(root).map_err(|e| e.to_string())?;
        if std::fs::symlink_metadata(root)
            .map_err(|e| e.to_string())?
            .file_type()
            .is_symlink()
        {
            return Err("Asset root must not be a symlink".into());
        }
        let root_real = std::fs::canonicalize(root).map_err(|e| e.to_string())?;
        let mut parent_real = root_real.clone();
        for component in asset
            .path
            .parent()
            .ok_or("Asset has no parent")?
            .components()
        {
            let Component::Normal(name) = component else {
                return Err("Invalid asset directory".into());
            };
            let next = parent_real.join(name);
            if !next.exists() {
                std::fs::create_dir(&next).map_err(|e| e.to_string())?;
            }
            parent_real = std::fs::canonicalize(next).map_err(|e| e.to_string())?;
            if !parent_real.starts_with(&root_real) {
                return Err("Asset destination escapes case directory".into());
            }
        }
        let path = parent_real.join(asset.path.file_name().ok_or("Asset has no file name")?);
        if std::fs::read(&path)
            .ok()
            .is_some_and(|bytes| bytes == content.as_bytes())
        {
            continue;
        }
        let mut file = tempfile::NamedTempFile::new_in(&parent_real).map_err(|e| e.to_string())?;
        file.write_all(content.as_bytes())
            .map_err(|e| e.to_string())?;
        file.persist(path).map_err(|e| e.to_string())?;
    }
    Ok(())
}
pub fn instantiate(
    entry: &CaseTemplateManifest,
    preset: Option<&str>,
) -> Result<WorkbenchProject, String> {
    entry.validate().map_err(|e| e.to_string())?;
    let mut project = entry.project_template.clone();
    project.runs.clear();
    project.mesh_asset = None;
    project.project_id = WorkbenchProject::blank(&entry.name).project_id;
    if let Some(id) = preset {
        let p = entry
            .metadata
            .presets
            .iter()
            .find(|p| p.id == id)
            .ok_or_else(|| format!("Unknown case preset: {id}"))?;
        if let Some(mesh) = &p.mesh {
            project.mesh = mesh.clone();
        }
        if let Some(recipe) = &p.mesh_recipe {
            project.mesh_recipe = recipe.clone();
        }
        if let Some(time) = p.time {
            project.solver.time = time;
            project.solver.physical_end_time = p.physical_end_time;
        }
        if let Some(time) = &p.compressible_time {
            let settings = project
                .solver
                .compressible
                .as_mut()
                .ok_or("Compressible preset on incompressible case")?;
            settings.courant = time.courant;
            settings.max_time_step = time.max_time_step;
            settings.end_time = time.end_time;
            settings.max_steps = time.max_steps;
        }
    }
    if let Some(section) = &entry.metadata.display.section {
        if !project.plane_monitors.iter().any(|p| p.id == section.id) {
            project.plane_monitors.push(section.clone());
        }
    }
    project.case_context = Some(CaseContext {
        id: entry.id.clone(),
        revision: entry.metadata.revision.clone(),
        selected_preset: preset.map(str::to_owned),
        metadata: entry.metadata.clone(),
    });
    project.validate().map_err(|e| e.to_string())?;
    Ok(project)
}
#[derive(Clone, Debug, Serialize)]
pub struct ErrorNorms {
    pub samples: usize,
    pub l1: f64,
    pub l2: f64,
    pub linf: f64,
    pub relative_l2: Option<f64>,
}
pub fn compare(observed: &[f64], expected: &[f64], weights: &[f64]) -> Result<ErrorNorms, String> {
    if observed.is_empty()
        || observed.len() != expected.len()
        || (!weights.is_empty() && weights.len() != observed.len())
    {
        return Err("Comparison arrays must have equal non-zero lengths".into());
    }
    let (mut sum, mut l1, mut l2, mut linf, mut reference) = (0_f64, 0_f64, 0_f64, 0_f64, 0_f64);
    for (i, (&a, &b)) in observed.iter().zip(expected).enumerate() {
        let w = weights.get(i).copied().unwrap_or(1.);
        if !a.is_finite() || !b.is_finite() || !w.is_finite() || w < 0. {
            return Err("Comparison requires finite samples and non-negative weights".into());
        }
        if w == 0. {
            continue;
        }
        let error = (a - b).abs();
        sum += w;
        l1 += w * error;
        l2 += w * error * error;
        reference += w * b * b;
        linf = linf.max(error);
    }
    if sum <= 0. || !sum.is_finite() || !l2.is_finite() || !reference.is_finite() {
        return Err("Comparison weights or norms are invalid".into());
    }
    Ok(ErrorNorms {
        samples: observed.len(),
        l1: l1 / sum,
        l2: (l2 / sum).sqrt(),
        linf,
        relative_l2: (reference > 0.).then(|| (l2 / reference).sqrt()),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn case_metadata_presets_preserve_independent_study_axes() {
        let cases = crate::workbench::catalogue::builtin_templates().expect("case library");
        assert_eq!(cases.len(), 27);
        for case in cases {
            assert!(!case.metadata.sources.is_empty());
            for preset in &case.metadata.presets {
                let project = instantiate(&case, Some(&preset.id)).expect("valid preset");
                let restored = crate::workbench::WorkbenchSession::from_project(&project)
                    .expect("preset session")
                    .to_project("restored");
                assert_eq!(restored.case_context, project.case_context);
                assert_eq!(
                    project.case_context.as_ref().map(|c| c.id.as_str()),
                    Some(case.id.as_str())
                );
                if preset.id.starts_with("time-") {
                    assert_eq!(project.mesh, case.project_template.mesh);
                    assert_eq!(project.mesh_recipe, case.project_template.mesh_recipe);
                } else {
                    assert_eq!(project.solver.time, case.project_template.solver.time);
                    assert_eq!(
                        project.solver.compressible,
                        case.project_template.solver.compressible
                    );
                }
            }
        }
    }
    #[test]
    fn case_metadata_assets_export_reload_and_detect_tampering() {
        let cases = crate::workbench::catalogue::builtin_templates().expect("case library");
        let case = cases
            .iter()
            .find(|c| c.id == "couette-planar")
            .expect("Couette");
        let directory = tempfile::tempdir().expect("temporary root");
        let path = crate::workbench::save_case_template(directory.path(), case)
            .expect("export with assets");
        let loaded =
            crate::workbench::catalogue::load_case_file(&path).expect("reload with assets");
        assert_eq!(loaded.metadata, case.metadata);
        let asset = path
            .parent()
            .expect("case parent")
            .join(&case.metadata.assets[0].path);
        std::fs::write(asset, "tampered reference").expect("tamper");
        assert!(crate::workbench::catalogue::load_case_file(&path)
            .expect_err("checksum rejection")
            .contains("checksum"));
        assert!(relative_asset(Path::new("../escape.csv")).is_err());
        assert!(relative_asset(Path::new("/tmp/escape.csv")).is_err());
    }
    #[test]
    fn case_metadata_old_v2_and_error_norms_remain_valid() {
        let text = include_str!("../../examples/cases/laminar-channel/case.json");
        let mut json: serde_json::Value = serde_json::from_str(text).expect("JSON");
        json.as_object_mut().expect("object").remove("metadata");
        let case: CaseTemplateManifest = serde_json::from_value(json).expect("old version 2");
        case.validate().expect("backward-compatible template");
        assert_eq!(case.metadata.revision, "1");
        assert_eq!(case.metadata.verification.status, "unvalidated");
        let error = compare(&[2., 4.], &[1., 2.], &[1., 3.]).expect("weighted norms");
        assert!((error.l1 - 1.75).abs() < 1e-12);
        assert!((error.l2 - (13_f64 / 4.).sqrt()).abs() < 1e-12);
        assert_eq!(error.relative_l2, Some(1.));
        assert_eq!(
            compare(&[1.], &[0.], &[])
                .expect("zero reference")
                .relative_l2,
            None
        );
        assert!(compare(&[1.], &[0.], &[0.]).is_err());
    }
}
