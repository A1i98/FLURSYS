use super::*;

/// Exact analytic B-rep primitives, in model units, with a Z-oriented axis.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum CadPrimitive {
    Box {
        length: f64,
        width: f64,
        height: f64,
    },
    Cylinder {
        radius: f64,
        height: f64,
    },
    Sphere {
        radius: f64,
    },
    Cone {
        radius: f64,
        height: f64,
    },
    Torus {
        major_radius: f64,
        minor_radius: f64,
    },
}

pub(super) fn positive(v: f64, label: &str) -> Result<(), GeometryError> {
    if v.is_finite() && v > 1e-8 && v < 1e9 {
        Ok(())
    } else {
        Err(error(format!(
            "{label} must be finite and between 1e-8 and 1e9 model units"
        )))
    }
}

pub(super) fn primitive(shape: CadPrimitive) -> Result<Solid, GeometryError> {
    use CadPrimitive::*;
    let origin = Point3::origin();
    let solid = match shape {
        Box {
            length,
            width,
            height,
        } => {
            for (v, n) in [(length, "Length"), (width, "Width"), (height, "Height")] {
                positive(v, n)?;
            }
            let v = builder::vertex(origin);
            let e = builder::tsweep(&v, Vector3::new(length, 0.0, 0.0));
            let f = builder::tsweep(&e, Vector3::new(0.0, width, 0.0));
            builder::tsweep(&f, Vector3::new(0.0, 0.0, height))
        }
        Cylinder { radius, height } => {
            positive(radius, "Radius")?;
            positive(height, "Height")?;
            let v = builder::vertex(Point3::new(radius, 0.0, 0.0));
            let circle = builder::rsweep(&v, origin, Vector3::unit_z(), Rad(std::f64::consts::TAU));
            let disk = builder::try_attach_plane(&[circle]).map_err(|e| error(e.to_string()))?;
            builder::tsweep(&disk, Vector3::new(0.0, 0.0, height))
        }
        Sphere { radius } => {
            positive(radius, "Radius")?;
            let v = builder::vertex(Point3::new(0.0, 0.0, radius));
            let arc = builder::rsweep(&v, origin, Vector3::unit_y(), Rad(std::f64::consts::PI));
            Solid::try_new(vec![builder::cone(
                &arc,
                Vector3::unit_z(),
                Rad(std::f64::consts::TAU),
            )])
            .map_err(|e| error(e.to_string()))?
        }
        Cone { radius, height } => {
            positive(radius, "Radius")?;
            positive(height, "Height")?;
            let a = builder::vertex(Point3::new(0.0, 0.0, height));
            let b = builder::vertex(Point3::new(radius, 0.0, 0.0));
            let c = builder::vertex(origin);
            let wire: Wire = vec![builder::line(&a, &b), builder::line(&b, &c)].into();
            Solid::try_new(vec![builder::cone(
                &wire,
                Vector3::unit_z(),
                Rad(std::f64::consts::TAU),
            )])
            .map_err(|e| error(e.to_string()))?
        }
        Torus {
            major_radius,
            minor_radius,
        } => {
            positive(major_radius, "Major radius")?;
            positive(minor_radius, "Minor radius")?;
            if major_radius <= minor_radius {
                return Err(error("Major radius must exceed minor radius"));
            }
            let v = builder::vertex(Point3::new(major_radius, 0.0, minor_radius));
            let w = builder::rsweep(
                &v,
                Point3::new(major_radius, 0.0, 0.0),
                Vector3::unit_y(),
                Rad(std::f64::consts::TAU),
            );
            Solid::try_new(vec![builder::rsweep(
                &w,
                origin,
                Vector3::unit_z(),
                Rad(std::f64::consts::TAU),
            )])
            .map_err(|e| error(e.to_string()))?
        }
    };
    Ok(solid)
}
