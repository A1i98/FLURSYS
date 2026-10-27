use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{io::Write, path::Path};

struct HashWriter(Sha256);
impl Write for HashWriter {
    fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
        self.0.update(bytes);
        Ok(bytes.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}
pub(crate) fn digest(value: &impl Serialize) -> Result<String, String> {
    let mut writer = HashWriter(Sha256::new());
    serde_json::to_writer(&mut writer, value).map_err(|e| e.to_string())?;
    Ok(format!("{:x}", writer.0.finalize()))
}
pub(crate) fn write_json(
    path: &Path,
    value: &impl Serialize,
    byte_limit: Option<u64>,
    before_replace: impl FnOnce() -> Result<(), String>,
) -> Result<u64, String> {
    let parent = path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    let mut temp = tempfile::NamedTempFile::new_in(parent).map_err(|e| e.to_string())?;
    {
        let mut writer = std::io::BufWriter::new(temp.as_file_mut());
        serde_json::to_writer(&mut writer, value).map_err(|e| e.to_string())?;
        writer.flush().map_err(|e| e.to_string())?;
    }
    let bytes = temp.as_file().metadata().map_err(|e| e.to_string())?.len();
    if byte_limit.is_some_and(|limit| limit > 0 && bytes > limit) {
        return Err(
            "A field-history frame exceeds the storage budget; recovery checkpoints remain enabled"
                .into(),
        );
    }
    temp.as_file().sync_all().map_err(|e| e.to_string())?;
    before_replace()?;
    temp.persist(path).map_err(|e| e.to_string())?;
    #[cfg(unix)]
    std::fs::File::open(parent)
        .and_then(|f| f.sync_all())
        .map_err(|e| e.to_string())?;
    Ok(bytes)
}
