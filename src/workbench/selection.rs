//! Stable geometry selection and Named Selection state for the workbench UI.
//!
//! UI selection identity is bound to stable geometry IDs, never to generated
//! mesh face indices. `NamedSelection` stores a deterministic, ordered set of
//! stable targets under a validated, unique name.

use crate::{BodyId, EdgeId, FaceId};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum GeometrySelectionTarget {
    Edge(EdgeId),
    Face(FaceId),
    Body(BodyId),
}

impl GeometrySelectionTarget {
    pub const fn kind_label(self) -> &'static str {
        match self {
            Self::Edge(_) => "edge",
            Self::Face(_) => "face",
            Self::Body(_) => "body",
        }
    }

    pub const fn id_value(self) -> u64 {
        match self {
            Self::Edge(id) => id.get(),
            Self::Face(id) => id.get(),
            Self::Body(id) => id.get(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct NamedSelection {
    pub name: String,
    pub targets: Vec<GeometrySelectionTarget>,
}

impl NamedSelection {
    /// True when the selection contains only targets of one kind.
    pub fn is_uniform(&self) -> bool {
        self.targets.iter().all(|target| {
            std::mem::discriminant(target) == std::mem::discriminant(&self.targets[0])
        })
    }

    pub fn edges(&self) -> Vec<EdgeId> {
        self.targets
            .iter()
            .filter_map(|target| match target {
                GeometrySelectionTarget::Edge(id) => Some(*id),
                _ => None,
            })
            .collect()
    }

    pub fn faces(&self) -> Vec<FaceId> {
        self.targets
            .iter()
            .filter_map(|target| match target {
                GeometrySelectionTarget::Face(id) => Some(*id),
                _ => None,
            })
            .collect()
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum NamedSelectionError {
    InvalidName { name: String },
    DuplicateName { name: String },
    EmptySelection,
    UnknownTarget { target: GeometrySelectionTarget },
    UnknownSelection { name: String },
}

impl std::fmt::Display for NamedSelectionError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidName { name } => {
                write!(
                    formatter,
                    "invalid Named Selection name {name:?}: use non-empty text without quotes or control characters"
                )
            }
            Self::DuplicateName { name } => {
                write!(formatter, "Named Selection {name:?} already exists")
            }
            Self::EmptySelection => {
                write!(
                    formatter,
                    "a Named Selection needs at least one geometry entity"
                )
            }
            Self::UnknownTarget { target } => write!(
                formatter,
                "{} {} does not exist in the current geometry",
                target.kind_label(),
                target.id_value()
            ),
            Self::UnknownSelection { name } => {
                write!(formatter, "Named Selection {name:?} does not exist")
            }
        }
    }
}

impl std::error::Error for NamedSelectionError {}

/// Deterministic insertion-ordered storage of Named Selections.
#[derive(Clone, Debug, Default)]
pub struct NamedSelectionStore {
    selections: Vec<NamedSelection>,
}

impl NamedSelectionStore {
    /// Creates a Named Selection after validating the name, duplicate names,
    /// empty selections, and that every target exists in the live topology.
    pub fn create(
        &mut self,
        name: &str,
        targets: Vec<GeometrySelectionTarget>,
        target_exists: impl Fn(GeometrySelectionTarget) -> bool,
    ) -> Result<(), NamedSelectionError> {
        let name = validate_name(name)?;
        if self.get(&name).is_some() {
            return Err(NamedSelectionError::DuplicateName { name });
        }
        let mut unique = targets;
        if unique.is_empty() {
            return Err(NamedSelectionError::EmptySelection);
        }
        unique.sort();
        unique.dedup();
        for &target in &unique {
            if !target_exists(target) {
                return Err(NamedSelectionError::UnknownTarget { target });
            }
        }
        self.selections.push(NamedSelection {
            name,
            targets: unique,
        });
        Ok(())
    }

    pub fn rename(&mut self, old_name: &str, new_name: &str) -> Result<(), NamedSelectionError> {
        let new_name = validate_name(new_name)?;
        if new_name != old_name && self.get(&new_name).is_some() {
            return Err(NamedSelectionError::DuplicateName { name: new_name });
        }
        let selection = self
            .selections
            .iter_mut()
            .find(|selection| selection.name == old_name)
            .ok_or_else(|| NamedSelectionError::UnknownSelection {
                name: old_name.to_string(),
            })?;
        selection.name = new_name;
        Ok(())
    }

    pub fn delete(&mut self, name: &str) -> bool {
        let before = self.selections.len();
        self.selections.retain(|selection| selection.name != name);
        self.selections.len() != before
    }

    pub fn get(&self, name: &str) -> Option<&NamedSelection> {
        self.selections
            .iter()
            .find(|selection| selection.name == name)
    }

    pub fn get_at(&self, index: usize) -> Option<&NamedSelection> {
        self.selections.get(index)
    }

    pub fn iter(&self) -> impl Iterator<Item = &NamedSelection> {
        self.selections.iter()
    }

    pub fn len(&self) -> usize {
        self.selections.len()
    }

    pub fn is_empty(&self) -> bool {
        self.selections.is_empty()
    }

    /// Names of every selection containing the given target, in storage order.
    pub fn membership(&self, target: GeometrySelectionTarget) -> Vec<&str> {
        self.selections
            .iter()
            .filter(|selection| selection.targets.contains(&target))
            .map(|selection| selection.name.as_str())
            .collect()
    }
}

fn validate_name(name: &str) -> Result<String, NamedSelectionError> {
    let trimmed = name.trim();
    let invalid = trimmed.is_empty()
        || trimmed.chars().any(|character| {
            character == '"' || character.is_control() || character.is_whitespace()
        });
    if invalid {
        return Err(NamedSelectionError::InvalidName {
            name: name.to_string(),
        });
    }
    Ok(trimmed.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::GeometryTopology;

    struct Fixture {
        first: GeometrySelectionTarget,
        second: GeometrySelectionTarget,
        third: GeometrySelectionTarget,
        topology: GeometryTopology,
    }

    fn fixture() -> Fixture {
        let mut topology = GeometryTopology::new();
        let rectangle = topology.add_rectangle(2.0, 1.0).unwrap();
        Fixture {
            first: GeometrySelectionTarget::Edge(rectangle.left),
            second: GeometrySelectionTarget::Edge(rectangle.right),
            third: GeometrySelectionTarget::Edge(rectangle.top),
            topology,
        }
    }

    #[test]
    fn create_validates_names_duplicates_and_targets() {
        let mut store = NamedSelectionStore::default();
        assert!(matches!(
            store.create("  ", vec![], |_| true),
            Err(NamedSelectionError::InvalidName { .. })
        ));
        assert!(matches!(
            store.create("in\"let", vec![], |_| true),
            Err(NamedSelectionError::InvalidName { .. })
        ));
        assert!(matches!(
            store.create("inlet", Vec::new(), |_| true),
            Err(NamedSelectionError::EmptySelection)
        ));
        let unknown = GeometrySelectionTarget::Face(crate::FaceId::default());
        assert!(matches!(
            store.create("inlet", vec![unknown], |_| false),
            Err(NamedSelectionError::UnknownTarget { target } ) if target == unknown
        ));

        let data = fixture();
        let exists = |target| {
            data.topology
                .edge(match target {
                    GeometrySelectionTarget::Edge(id) => id,
                    _ => unreachable!("edge-only fixture"),
                })
                .is_some()
        };
        let two = vec![data.second, data.first];
        store.create("inlet", two.clone(), exists).unwrap();
        store.create("walls", vec![data.third], exists).unwrap();
        assert!(matches!(
            store.create(" inlet ", vec![data.third], exists),
            Err(NamedSelectionError::DuplicateName { .. })
        ));
        let inlet = store.get("inlet").unwrap();
        assert_eq!(inlet.targets, vec![data.second, data.first]); // sorted by id
        assert_eq!(two.len(), 2);
    }

    #[test]
    fn rename_delete_and_membership_are_deterministic() {
        let data = fixture();
        let mut store = NamedSelectionStore::default();
        let exists = |target| {
            data.topology
                .edge(match target {
                    GeometrySelectionTarget::Edge(id) => id,
                    _ => unreachable!("edge-only fixture"),
                })
                .is_some()
        };
        store.create("inlet", vec![data.first], exists).unwrap();
        store
            .create("walls", vec![data.first, data.second], exists)
            .unwrap();
        assert_eq!(store.membership(data.first), vec!["inlet", "walls"]);
        assert!(matches!(
            store.rename("missing", "x"),
            Err(NamedSelectionError::UnknownSelection { .. })
        ));
        store.rename("walls", "boundary").unwrap();
        assert!(store.get("walls").is_none());
        assert!(store.get("boundary").is_some());
        assert!(store.delete("inlet"));
        assert!(!store.delete("inlet"));
    }
}
