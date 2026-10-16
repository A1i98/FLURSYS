//! Analytic finishing of rectangular prisms with persistent local selections.
use super::*;

#[derive(Clone, Copy)]
struct Prism {
    origin: Vec3,
    x: Vec3,
    y: Vec3,
    z: Vec3,
    l: f64,
    w: f64,
    h: f64,
}
impl Prism {
    fn point(self, x: f64, y: f64, z: f64) -> Point3 {
        point(self.origin + self.x * x + self.y * y + self.z * z)
    }
    fn transformed(self, t: CadTransform) -> Result<Self, GeometryError> {
        let apply = |p: Vec3| -> Vec3 {
            match t {
                CadTransform::Translate(v) => p + v,
                CadTransform::Rotate {
                    pivot,
                    axis,
                    degrees,
                } => {
                    let n = axis * (1. / axis.norm());
                    let q = p - pivot;
                    let a = degrees.to_radians();
                    pivot + q * a.cos() + n.cross(q) * a.sin() + n * (n.dot(q) * (1. - a.cos()))
                }
                CadTransform::Mirror { pivot, normal } => {
                    let n = normal * (1. / normal.norm());
                    p - n * (2. * (p - pivot).dot(n))
                }
                CadTransform::Scale { pivot, factor } => pivot + (p - pivot) * factor,
                CadTransform::ScaleAxes { pivot, factors } => {
                    let q = p - pivot;
                    pivot + Vec3::new(q.x * factors.x, q.y * factors.y, q.z * factors.z)
                }
            }
        };
        let o = apply(self.origin);
        let x = apply(self.origin + self.x * self.l) - o;
        let y = apply(self.origin + self.y * self.w) - o;
        let z = apply(self.origin + self.z * self.h) - o;
        let (l, w, h) = (x.norm(), y.norm(), z.norm());
        for (v, label) in [(l, "Length"), (w, "Width"), (h, "Depth")] {
            primitives::positive(v, label)?;
        }
        let (x, y, z) = (x * (1. / l), y * (1. / w), z * (1. / h));
        if x.dot(y).abs() > 1e-7 || y.dot(z).abs() > 1e-7 || z.dot(x).abs() > 1e-7 {
            return Err(error(
                "Finishing cannot reconstruct a skewed prism after nonuniform scaling",
            ));
        }
        finite(o)?;
        Ok(if x.cross(y).dot(z) > 0. {
            Self {
                origin: o,
                x,
                y,
                z,
                l,
                w,
                h,
            }
        } else {
            Self {
                origin: o,
                x: y,
                y: x,
                z,
                l: w,
                w: l,
                h,
            }
        })
    }
    fn open(self, side: u8) -> Self {
        let p = self;
        match side {
            0 => Self {
                origin: p.origin + p.x * p.l,
                x: p.z,
                y: p.y,
                z: p.x * (-1.),
                l: p.h,
                w: p.w,
                h: p.l,
            },
            1 => Self {
                x: p.y,
                y: p.z,
                z: p.x,
                l: p.w,
                w: p.h,
                h: p.l,
                ..p
            },
            2 => Self {
                origin: p.origin + p.y * p.w,
                x: p.x,
                y: p.z,
                z: p.y * (-1.),
                l: p.l,
                w: p.h,
                h: p.w,
            },
            3 => Self {
                x: p.z,
                y: p.x,
                z: p.y,
                l: p.h,
                w: p.l,
                h: p.w,
                ..p
            },
            4 => Self {
                origin: p.origin + p.z * p.h,
                x: p.y,
                y: p.x,
                z: p.z * (-1.),
                l: p.w,
                w: p.l,
                h: p.h,
            },
            _ => p,
        }
    }
}
impl GeometryTopology {
    fn finishing_prism(&self, source: BodyId) -> Result<Prism, GeometryError> {
        let recipe = self
            .cad_body(source)
            .and_then(|b| b.recipe.as_ref())
            .ok_or_else(|| error("Body has no finishing construction recipe"))?;
        self.finishing_recipe(recipe, 0)
    }
    fn finishing_recipe(&self, recipe: &CadRecipe, depth: usize) -> Result<Prism, GeometryError> {
        if depth > 64 {
            return Err(error("Finishing source chain exceeds 64 operations"));
        }
        match recipe {
            CadRecipe::Primitive{shape:CadPrimitive::Box{length,width,height},position}=>Ok(Prism{origin:*position,x:Vec3::new(1.,0.,0.),y:Vec3::new(0.,1.,0.),z:Vec3::new(0.,0.,1.),l:*length,w:*width,h:*height}),
            CadRecipe::Extrude{face,distance}=>{
                let sketch=self.sketches().find(|s|s.profile_face==Some(*face)).ok_or_else(||error("Finishing requires a saved rectangular sketch"))?;
                let (x,y,profile)=self.cad_sketch_parameters(sketch.id)?;
                let CadSketchProfile::Rectangle{width,height}=profile else {return Err(error("This finishing tool requires a rectangular Pad"));};
                let f=sketch.frame;
                Ok(Prism{origin:f.origin+f.x_axis*x+f.y_axis*y+f.normal*distance.min(0.),x:f.x_axis,y:f.y_axis,z:f.normal,l:width,w:height,h:distance.abs()})
            }
            CadRecipe::Modified{source,transform}=>self.finishing_recipe(source,depth+1)?.transformed(*transform),
            CadRecipe::Transform{source,transform}=>{
                let recipe=self.cad_body(*source).and_then(|b|b.recipe.as_ref()).ok_or_else(||error("Finishing transform source has no construction recipe"))?;
                self.finishing_recipe(recipe,depth+1)?.transformed(*transform)
            }
            _=>Err(error("Supported sources: Box primitives and rectangular Pads on any sketch plane. General curved-body blending is not available.")),
        }
    }
    /// Validate picks and convert unstable edge IDs into local construction roles.
    /// Empty edge selection means all four longitudinal edges; absent face means +Z.
    pub fn cad_finishing_selection(
        &self,
        source: BodyId,
        edges: &[EdgeId],
        face: Option<FaceId>,
    ) -> Result<(u8, u8), GeometryError> {
        let p = self.finishing_prism(source)?;
        let mut mask = if edges.is_empty() { 15 } else { 0 };
        for &id in edges {
            if self.cad_edge_owner(id) != Some(source) {
                return Err(error("Every selected edge must belong to the source body"));
            }
            let (a, d) = self.cad_reference_axis(CadAxisReference::Edge(id))?;
            let q = a - p.origin;
            let tolerance = 1e-7 * p.l.max(p.w).max(p.h).max(1.);
            if d.cross(p.z).norm() > 1e-7 * d.norm() || (d.norm() - p.h).abs() > tolerance {
                return Err(error(
                    "Select longitudinal prism edges. Cap edges are not supported by this tool.",
                ));
            }
            let x = q.dot(p.x);
            let y = q.dot(p.y);
            let corner = if x.abs() < tolerance && y.abs() < tolerance {
                0
            } else if (x - p.l).abs() < tolerance && y.abs() < tolerance {
                1
            } else if (x - p.l).abs() < tolerance && (y - p.w).abs() < tolerance {
                2
            } else if x.abs() < tolerance && (y - p.w).abs() < tolerance {
                3
            } else {
                return Err(error("Selected edge is not a prism corner"));
            };
            mask |= 1 << corner;
        }
        let opening = if let Some(face) = face {
            if self.cad_owner(face) != Some(source) {
                return Err(error("Shell opening must belong to the source body"));
            }
            let f = self.sketch_frame_for_face(face)?;
            let q = f.origin - p.origin;
            let distances = [
                q.dot(p.x).abs(),
                (q.dot(p.x) - p.l).abs(),
                q.dot(p.y).abs(),
                (q.dot(p.y) - p.w).abs(),
                q.dot(p.z).abs(),
                (q.dot(p.z) - p.h).abs(),
            ];
            let i = (0..6)
                .filter(|&i| f.normal.dot([p.x, p.x, p.y, p.y, p.z, p.z][i]).abs() > 0.999999)
                .min_by(|&a, &b| distances[a].total_cmp(&distances[b]))
                .ok_or_else(|| error("Choose a planar prism face"))?;
            if distances[i] > 1e-7 * p.l.max(p.w).max(p.h).max(1.) {
                return Err(error("Opening face is not on the prism boundary"));
            }
            i as u8
        } else {
            5
        };
        Ok((mask, opening))
    }
    pub fn cad_finishing_maximum(
        &self,
        source: BodyId,
        kind: u8,
        opening: u8,
    ) -> Result<f64, GeometryError> {
        let base = self.finishing_prism(source)?;
        let p = if kind == 2 { base.open(opening) } else { base };
        Ok(if kind == 2 {
            (p.l.min(p.w) * 0.5).min(p.h)
        } else {
            p.l.min(p.w) * 0.5
        })
    }
    pub fn cad_finish_box(
        &mut self,
        source: BodyId,
        kind: u8,
        amount: f64,
    ) -> Result<BodyId, GeometryError> {
        self.cad_finish_prism(source, kind, amount, 15, 5)
    }
    pub fn cad_finish_prism(
        &mut self,
        source: BodyId,
        kind: u8,
        amount: f64,
        corners: u8,
        opening: u8,
    ) -> Result<BodyId, GeometryError> {
        primitives::positive(amount, "Radius / thickness")?;
        if kind > 2 || corners == 0 || corners > 15 || opening > 5 {
            return Err(error("Invalid finishing selection"));
        }
        let base = self.finishing_prism(source)?;
        let p = if kind == 2 { base.open(opening) } else { base };
        let (l, w, h) = (p.l, p.w, p.h);
        if amount * 2. >= l.min(w) || (kind == 2 && amount >= h) {
            return Err(error("Radius / thickness must be below half the smaller profile dimension; floor thickness must be below depth"));
        }
        let r = amount;
        let solid = if kind <= 1 {
            let coordinates = [(0., 0.), (l, 0.), (l, w), (0., w)];
            let dirs = [
                ((0., 1.), (1., 0.)),
                ((-1., 0.), (0., 1.)),
                ((0., -1.), (-1., 0.)),
                ((1., 0.), (0., -1.)),
            ];
            let mut entries = Vec::new();
            let mut exits = Vec::new();
            for i in 0..4 {
                let (x, y) = coordinates[i];
                let (a, b) = dirs[i];
                if corners & (1 << i) != 0 {
                    entries.push(builder::vertex(p.point(x + a.0 * r, y + a.1 * r, 0.)));
                    exits.push(builder::vertex(p.point(x + b.0 * r, y + b.1 * r, 0.)));
                } else {
                    let v = builder::vertex(p.point(x, y, 0.));
                    entries.push(v.clone());
                    exits.push(v);
                }
            }
            let mut edges = Vec::new();
            for i in 0..4 {
                if corners & (1 << i) != 0 {
                    if kind == 1 {
                        edges.push(builder::line(&entries[i], &exits[i]));
                    } else {
                        let (x, y) = coordinates[i];
                        let (a, b) = dirs[i];
                        let k = r * (1. - std::f64::consts::FRAC_1_SQRT_2);
                        edges.push(builder::circle_arc(
                            &entries[i],
                            &exits[i],
                            p.point(x + (a.0 + b.0) * k, y + (a.1 + b.1) * k, 0.),
                        ));
                    }
                }
                edges.push(builder::line(&exits[i], &entries[(i + 1) % 4]));
            }
            let wire: Wire = edges.into();
            let face = builder::try_attach_plane(&[wire]).map_err(|e| error(e.to_string()))?;
            builder::tsweep(&face, vector(p.z * h))
        } else {
            // Shared edges are reused in opposite directions by adjacent faces.
            let vertices: Vec<_> = [
                (0., 0., 0.),
                (l, 0., 0.),
                (l, w, 0.),
                (0., w, 0.),
                (0., 0., h),
                (l, 0., h),
                (l, w, h),
                (0., w, h),
                (r, r, r),
                (l - r, r, r),
                (l - r, w - r, r),
                (r, w - r, r),
                (r, r, h),
                (l - r, r, h),
                (l - r, w - r, h),
                (r, w - r, h),
            ]
            .iter()
            .map(|&(x, y, z)| builder::vertex(p.point(x, y, z)))
            .collect();
            let mut edges = std::collections::BTreeMap::new();
            let mut loop_wire = |indices: &[usize]| -> Wire {
                (0..indices.len())
                    .map(|i| {
                        let a = indices[i];
                        let b = indices[(i + 1) % indices.len()];
                        let edge = edges.entry((a.min(b), a.max(b))).or_insert_with(|| {
                            builder::line(&vertices[a.min(b)], &vertices[a.max(b)])
                        });
                        if a < b {
                            edge.clone()
                        } else {
                            edge.inverse()
                        }
                    })
                    .collect()
            };
            let polygons = [
                vec![0, 3, 2, 1],
                vec![0, 1, 5, 4],
                vec![1, 2, 6, 5],
                vec![2, 3, 7, 6],
                vec![3, 0, 4, 7],
                vec![8, 9, 10, 11],
                vec![8, 12, 13, 9],
                vec![9, 13, 14, 10],
                vec![10, 14, 15, 11],
                vec![11, 15, 12, 8],
            ];
            let mut faces = Vec::new();
            for polygon in polygons {
                faces.push(
                    builder::try_attach_plane(&[loop_wire(&polygon)])
                        .map_err(|e| error(e.to_string()))?,
                );
            }
            faces.push(
                builder::try_attach_plane(&[
                    loop_wire(&[4, 5, 6, 7]),
                    loop_wire(&[12, 15, 14, 13]),
                ])
                .map_err(|e| error(e.to_string()))?,
            );
            Solid::try_new(vec![faces.into()]).map_err(|e| error(e.to_string()))?
        };
        if !solid.is_geometric_consistent() {
            return Err(error("Finishing generated inconsistent geometry"));
        }
        let name = match kind {
            0 => "Prism fillet",
            1 => "Prism chamfer",
            _ => "Prism shell",
        };
        let id = self.cad_insert(solid, name, name)?;
        let body = self.cad_bodies.get_mut(&id).unwrap();
        body.dependencies = Some(CadDependencies {
            bodies: vec![source],
            profiles: vec![],
            replayable_pad: false,
        });
        body.recipe = Some(CadRecipe::Finish {
            source,
            kind,
            amount,
            corners,
            opening,
        });
        self.cad_bodies.get_mut(&source).unwrap().visible = false;
        self.name_constructed_faces(id)?;
        Ok(id)
    }
}
