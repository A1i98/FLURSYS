use crate::{compressible, IncompressibleCase, IncompressibleSolution, UnstructuredMesh};
#[derive(Clone, Debug)]
#[allow(clippy::large_enum_variant)]
pub enum FlowCase {
    Incompressible(IncompressibleCase),
    Compressible {
        mesh: UnstructuredMesh,
        settings: compressible::Settings,
    },
}
impl FlowCase {
    pub fn mesh(&self) -> &UnstructuredMesh {
        match self {
            Self::Incompressible(case) => &case.mesh,
            Self::Compressible { mesh, .. } => mesh,
        }
    }
    pub fn budget(&self) -> usize {
        match self {
            Self::Incompressible(case) => case
                .solver
                .time
                .parameters()
                .map(|(_, n)| n)
                .unwrap_or(case.solver.max_outer_iterations),
            Self::Compressible { settings, .. } => settings.max_steps,
        }
    }
}
#[derive(Clone, Debug)]
#[allow(clippy::large_enum_variant)]
pub enum FlowSolution {
    Incompressible(IncompressibleSolution),
    Compressible(compressible::Solution),
}
impl FlowSolution {
    pub fn dataset(
        &self,
        name: impl Into<String>,
        mesh: &UnstructuredMesh,
        gas: Option<compressible::IdealGas>,
    ) -> Result<crate::workbench::ResultDataset, String> {
        match self {
            Self::Incompressible(s) => crate::workbench::ResultDataset::from_values(
                name,
                mesh,
                s.pressure.values().to_vec(),
                s.velocity.values().to_vec(),
            )
            .and_then(|d| d.with_temperature(s.temperature.as_ref().map(|t| t.values().to_vec())))
            .map(|d| d.with_turbulence(s.turbulence.as_ref()))
            .map_err(|e| e.to_string()),
            Self::Compressible(s) => crate::workbench::ResultDataset::from_compressible(
                name,
                mesh,
                &s.fields,
                gas.ok_or("Missing gas model")?,
            )
            .map_err(|e| e.to_string()),
        }
    }
}
