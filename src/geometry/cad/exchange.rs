use super::*;
impl GeometryTopology {
    /// STEP preserves exact curves and surfaces; no renderer triangles are exported.
    pub fn cad_export_step(&self, id: BodyId) -> Result<String, GeometryError> {
        let solid = self.cad_solid(id)?;
        let compressed = solid.compress();
        Ok(truck_stepio::out::CompleteStepDisplay::new(
            truck_stepio::out::StepModel::from(&compressed),
            truck_stepio::out::StepHeaderDescriptor {
                organization_system: "FLURSYS".into(),
                ..Default::default()
            },
        )
        .to_string())
    }
    pub fn cad_export_brep(&self, id: BodyId) -> Result<String, GeometryError> {
        let solid = self.cad_solid(id)?;
        serde_json::to_string_pretty(&solid).map_err(|e| error(e.to_string()))
    }
    pub fn cad_import_brep(&mut self, text: &str, name: &str) -> Result<BodyId, GeometryError> {
        if text.len() > 64 * 1024 * 1024 {
            return Err(error("B-rep input exceeds 64 MiB"));
        }
        let solid: Solid = serde_json::from_str(text)
            .map_err(|e| error(format!("Invalid Truck B-rep file: {e}")))?;
        self.cad_insert(solid, name, "Imported B-rep")
    }
    pub fn cad_export_stl(&self, id: BodyId) -> Result<String, GeometryError> {
        let body = self
            .cad_body(id)
            .ok_or_else(|| error("Select a CAD body"))?;
        let mut result = String::from("solid flursys\n");
        use std::fmt::Write;
        for mesh in body.face_meshes.iter() {
            for tri in &mesh.triangles {
                let normal = (tri[1] - tri[0]).cross(tri[2] - tri[0]);
                let n = normal * (1.0 / normal.norm().max(1e-30));
                writeln!(result, "facet normal {} {} {}\nouter loop", n.x, n.y, n.z).unwrap();
                for v in tri {
                    writeln!(result, "vertex {} {} {}", v.x, v.y, v.z).unwrap();
                }
                result.push_str("endloop\nendfacet\n");
            }
        }
        result.push_str("endsolid flursys\n");
        Ok(result)
    }
}

impl GeometryTopology {
    /// Imports a single closed STEP shell with supported analytic/NURBS geometry.
    /// Assemblies, void-shell grouping and unsupported surfaces are rejected as
    /// a whole rather than silently importing an incomplete model.
    pub fn cad_import_step(&mut self, text: &str, name: &str) -> Result<BodyId, GeometryError> {
        use truck_stepio::r#in::{alias, Table};
        if text.len() > 64 * 1024 * 1024 {
            return Err(error("STEP input exceeds 64 MiB"));
        }
        let upper = text.to_ascii_uppercase();
        if [
            "BREP_WITH_VOIDS",
            "MAPPED_ITEM",
            "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION",
        ]
        .iter()
        .any(|kind| upper.contains(kind))
        {
            return Err(error(
                "STEP assemblies, mapped placements and void-shell grouping are not supported yet",
            ));
        }
        let table =
            Table::from_step(text).ok_or_else(|| error("Invalid or incomplete STEP document"))?;
        if table.shell.len() != 1 {
            return Err(error(
                "STEP import currently requires exactly one closed shell",
            ));
        }
        let shell = table
            .to_compressed_shell(table.shell.values().next().unwrap())
            .map_err(|e| error(format!("STEP geometry conversion failed: {e}")))?;
        let mut value = serde_json::to_value(&shell).map_err(|e| error(e.to_string()))?;
        for (i, edge) in shell.edges.iter().enumerate() {
            let curve = serde_json::to_value(&edge.curve).map_err(|e| error(e.to_string()))?;
            let _: truck_modeling::Curve = serde_json::from_value(curve.clone())
                .map_err(|_| error("STEP curve type is not supported; use B-spline/NURBS edges"))?;
            value["edges"][i]["curve"] = curve;
        }
        for (i, face) in shell.faces.iter().enumerate() {
            let surface = match &face.surface {
                alias::Surface::BSplineSurface(s) => serde_json::json!({"BSplineSurface":s}),
                alias::Surface::NurbsSurface(s) => serde_json::json!({"NurbsSurface":s}),
                alias::Surface::ElementarySurface(s) => {
                    match s.as_ref() {
                        alias::ElementarySurface::Plane(p) => serde_json::json!({"Plane":p}),
                        _ => return Err(error(
                            "This STEP elementary surface is not supported yet; use NURBS surfaces",
                        )),
                    }
                }
                alias::Surface::SweptCurve(s) => match s.as_ref() {
                    alias::SweptCurve::RevolutedCurve(p) => serde_json::json!({"RevolutedCurve":p}),
                    _ => return Err(error("This STEP swept surface is not supported yet")),
                },
            };
            let _: truck_modeling::Surface = serde_json::from_value(surface.clone())
                .map_err(|e| error(format!("Unsupported STEP surface: {e}")))?;
            value["faces"][i]["surface"] = surface;
        }
        let shell: truck_modeling::Shell = serde_json::from_value(value)
            .map_err(|e| error(format!("Invalid STEP shell topology: {e}")))?;
        let solid = Solid::try_new(vec![shell])
            .map_err(|e| error(format!("STEP shell is not a closed solid: {e}")))?;
        self.cad_insert(solid, name, "Imported STEP solid")
    }
}
