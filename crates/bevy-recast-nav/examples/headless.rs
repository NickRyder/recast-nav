use bevy_app::App;
use bevy_recast_nav::{NavigationSettings, NavigationWorld, RecastNavigationPlugin, TriangleMesh};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut app = App::new();
    app.add_plugins(RecastNavigationPlugin);
    let mesh = TriangleMesh {
        vertices: vec![[0., 0., 0.], [0., 0., 16.], [16., 0., 16.], [16., 0., 0.]],
        triangles: vec![[0, 1, 2], [0, 2, 3]],
    };
    let mut world = app.world_mut().resource_mut::<NavigationWorld>();
    world.rebuild(&mesh, NavigationSettings::default())?;
    let nav = world.get().unwrap();
    let path = nav.path([2., 0., 2.], [14., 0., 14.], false)?.unwrap();
    println!(
        "Built {} navigation polygons; {} waypoints; reaches goal: {}",
        nav.debug_polygons().len(),
        path.points.len(),
        path.reaches_goal
    );
    Ok(())
}
