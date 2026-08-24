//! CAD hover and selection state derived from canonical geometry ray picking.

use crate::{GeometryTopology, Ray3};

use super::GeometrySelectionTarget;

/// Explicit CAD selection interpretation for ray hits.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum CadPickMode {
    #[default]
    Face,
    Body,
}

/// Transient CAD interaction state for renderer hover and selected highlights.
///
/// It stores only stable canonical targets and is intentionally not persisted in
/// the workbench project document.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CadSelectionState {
    mode: CadPickMode,
    hover: Option<GeometrySelectionTarget>,
    selected: Option<GeometrySelectionTarget>,
}

impl CadSelectionState {
    pub const fn new(mode: CadPickMode) -> Self {
        Self {
            mode,
            hover: None,
            selected: None,
        }
    }

    pub const fn mode(&self) -> CadPickMode {
        self.mode
    }

    pub fn set_mode(&mut self, mode: CadPickMode) {
        self.mode = mode;
        self.hover = None;
    }

    pub const fn hover(&self) -> Option<GeometrySelectionTarget> {
        self.hover
    }

    pub const fn selected(&self) -> Option<GeometrySelectionTarget> {
        self.selected
    }

    /// Updates the transient hover target from the nearest visible CAD face.
    pub fn update_hover(
        &mut self,
        geometry: &GeometryTopology,
        ray: Ray3,
    ) -> Option<GeometrySelectionTarget> {
        self.hover = match self.mode {
            CadPickMode::Face => geometry
                .pick_face(ray)
                .map(|hit| GeometrySelectionTarget::Face(hit.face)),
            CadPickMode::Body => geometry.pick_body(ray).map(GeometrySelectionTarget::Body),
        };
        self.hover
    }

    /// Promotes the current hover target to the selected highlight target.
    pub fn select_hover(&mut self) -> Option<GeometrySelectionTarget> {
        self.selected = self.hover;
        self.selected
    }

    pub fn clear_hover(&mut self) {
        self.hover = None;
    }

    pub fn clear_selected(&mut self) {
        self.selected = None;
    }
}
