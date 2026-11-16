use recast_nav::{BuildConfig, TriangleMesh, build_tile};

#[test]
fn generation_errors_preserve_context_and_source_chains() {
    use recast_nav::{FilterError, RasterizationError, SpanInsertionError};
    use std::error::Error;

    let transparent = RasterizationError::from(FilterError::HeightOverflow);
    assert_eq!(
        transparent.to_string(),
        "native ledge height arithmetic overflows"
    );
    assert!(transparent.source().is_none());

    let contextual =
        RasterizationError::from(SpanInsertionError::ColumnIndexOutOfBounds { x: -1, y: 7 });
    assert_eq!(
        contextual.to_string(),
        "Failed to add span: column index out of bounds: x=-1, y=7"
    );
    assert!(contextual.source().unwrap().is::<SpanInsertionError>());
}

#[test]
fn invalid_geometry_and_configuration_return_errors() {
    let mesh = TriangleMesh {
        vertices: vec![[0., 0., 0.], [0., 0., 16.], [16., 0., 16.]],
        triangles: vec![[0, 1, 2]],
    };
    for config in [
        BuildConfig {
            max_vertices_per_polygon: u32::MAX,
            ..Default::default()
        },
        BuildConfig {
            max_edge_len: i32::MAX,
            ..Default::default()
        },
        BuildConfig {
            cell_size: 0.,
            ..Default::default()
        },
        BuildConfig {
            cell_height: f32::NAN,
            ..Default::default()
        },
        BuildConfig {
            max_grid_cells: 1,
            ..Default::default()
        },
        BuildConfig {
            border_size: 16,
            ..Default::default()
        },
        BuildConfig {
            slope_degrees: 90.,
            ..Default::default()
        },
        BuildConfig {
            bounds: [[0., -1., 0.], [16., f32::INFINITY, 16.]],
            ..Default::default()
        },
    ] {
        assert!(build_tile(&mesh, &config).is_err());
    }
    let mut invalid = mesh.clone();
    invalid.triangles[0][0] = 3;
    assert!(build_tile(&invalid, &BuildConfig::default()).is_err());
    invalid = mesh;
    invalid.vertices[0][1] = f32::INFINITY;
    assert!(build_tile(&invalid, &BuildConfig::default()).is_err());
}
