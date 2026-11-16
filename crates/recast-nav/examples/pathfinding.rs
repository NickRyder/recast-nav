use recast_nav::{
    BuildError, TriangleMesh,
    runtime::{Navigation, NavigationSettings},
};

fn main() -> Result<(), BuildError> {
    let mesh = TriangleMesh {
        vertices: vec![[0., 0., 0.], [0., 0., 16.], [16., 0., 16.], [16., 0., 0.]],
        triangles: vec![[0, 1, 2], [0, 2, 3]],
    };
    let navigation = Navigation::build(&mesh, NavigationSettings::default())?;
    match navigation.path([2., 0., 2.], [14., 0., 14.], false)? {
        Some(path) => {
            println!("Route with {} waypoints:", path.points.len());
            for point in path.points {
                println!("  {point:?}");
            }
        }
        None => println!("No complete route found."),
    }
    Ok(())
}
