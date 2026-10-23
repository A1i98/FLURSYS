//! Shared scalar-series figures. Missing or non-finite samples break curves.
use std::fmt::Write;
#[derive(Clone, Debug)]
pub struct Series {
    pub name: String,
    pub points: Vec<(f64, Option<f64>)>,
}
fn escape(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}
pub fn svg(series: &[Series], title: &str, x_label: &str, y_label: &str) -> Result<String, String> {
    if series.is_empty() || series.len() > 8 {
        return Err("Choose between one and eight curves".into());
    }
    let mut low = [f64::INFINITY; 2];
    let mut high = [f64::NEG_INFINITY; 2];
    for curve in series {
        for &(x, y) in &curve.points {
            if let Some(y) = y.filter(|v| v.is_finite()) {
                if x.is_finite() {
                    low[0] = low[0].min(x);
                    high[0] = high[0].max(x);
                    low[1] = low[1].min(y);
                    high[1] = high[1].max(y);
                }
            }
        }
    }
    if low.iter().chain(&high).any(|v| !v.is_finite()) {
        return Err("No finite curve samples".into());
    }
    for i in 0..2 {
        if low[i] == high[i] {
            let d = low[i].abs().max(1.0) * 0.05;
            low[i] -= d;
            high[i] += d;
        }
    }
    if (0..2).any(|i| !(high[i] - low[i]).is_finite()) {
        return Err("Plot range overflow".into());
    }
    let top = 78.0 + series.len().div_ceil(2) as f64 * 20.0;
    let mut out = format!(
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="650"><rect width="100%" height="100%" fill="#20242c"/><g font-family="DejaVu Sans,sans-serif" font-size="12" fill="#e0e6ef"><text x="110" y="32" font-size="18">{}</text>"##,
        escape(title)
    );
    let colors = [
        "#58c4f6", "#f5b85b", "#65d49c", "#c49bff", "#ff789b", "#e2e267", "#79ddd2", "#d1ac91",
    ];
    for (i, curve) in series.iter().enumerate() {
        let x = 110. + (i % 2) as f64 * 460.;
        let y = 58. + (i / 2) as f64 * 20.;
        let label = curve.name.chars().take(65).collect::<String>();
        let _ = write!(
            out,
            r##"<path d="M{x} {y} h22" stroke="{}" stroke-width="2"/><text x="{}" y="{}">{}</text>"##,
            colors[i],
            x + 28.,
            y + 4.,
            escape(&label)
        );
    }
    for i in 0..=5 {
        let t = i as f64 / 5.;
        let x = 110. + 940. * t;
        let y = 550. - (550. - top) * t;
        let _ = write!(
            out,
            r##"<path d="M{x} {top} V550 M110 {y} H1050" stroke="#3b4350"/><text x="{x}" y="574" text-anchor="middle">{:.4e}</text><text x="100" y="{}" text-anchor="end">{:.4e}</text>"##,
            low[0] + (high[0] - low[0]) * t,
            y + 4.,
            low[1] + (high[1] - low[1]) * t
        );
    }
    for (i, curve) in series.iter().enumerate() {
        let mut path = String::new();
        let mut drawing = false;
        for &(x, y) in &curve.points {
            if let Some(y) = y.filter(|v| v.is_finite() && x.is_finite()) {
                let sx = 110. + 940. * (x - low[0]) / (high[0] - low[0]);
                let sy = 550. - (550. - top) * (y - low[1]) / (high[1] - low[1]);
                let _ = write!(path, "{} {sx:.3} {sy:.3} ", if drawing { "L" } else { "M" });
                drawing = true;
            } else {
                drawing = false;
            }
        }
        let _ = write!(
            out,
            r##"<path d="{path}" fill="none" stroke="{}" stroke-width="2"/>"##,
            colors[i]
        );
    }
    let _ = write!(
        out,
        r##"<path d="M110 {top} V550 H1050" fill="none" stroke="#c3cedc"/><text x="580" y="610" text-anchor="middle">{}</text><text transform="translate(22 330) rotate(-90)" text-anchor="middle">{}</text></g></svg>"##,
        escape(x_label),
        escape(y_label)
    );
    Ok(out)
}
pub fn csv(series: &[Series], x_label: &str) -> String {
    let mut out = format!("series,{},value\n", x_label.replace(',', "_"));
    for curve in series {
        for (x, y) in &curve.points {
            let _ = writeln!(
                out,
                "\"{}\",{:.17e},{}",
                curve.name.replace('"', "\"\""),
                x,
                y.filter(|v| v.is_finite())
                    .map(|v| format!("{v:.17e}"))
                    .unwrap_or_default()
            );
        }
    }
    out
}
