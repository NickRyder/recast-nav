//! Headless Bevy 0.20 integration for the Rust Recast/Detour navigation core.
#![forbid(unsafe_code)]
use bevy_app::{App, Plugin};
use bevy_ecs::prelude::Resource;
pub use recast_nav::runtime::{BoxObstacle, Navigation, NavigationSettings, Path, UpdateProgress};
pub use recast_nav::{BuildConfig, BuildError, TriangleMesh};

pub struct RecastNavigationPlugin;
impl Plugin for RecastNavigationPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<NavigationWorld>();
    }
}

/// Navigation owned by a Bevy world. Generation can also run on a caller-owned task pool;
/// completed values are `Send + Sync` and can be inserted without native thread affinity.
#[derive(Resource, Default)]
pub struct NavigationWorld {
    navigation: Option<Navigation>,
}
impl NavigationWorld {
    /// Build and atomically replace the world. A failed build leaves the current world intact.
    pub fn rebuild(
        &mut self,
        mesh: &TriangleMesh,
        settings: NavigationSettings,
    ) -> Result<(), BuildError> {
        let navigation = Navigation::build(mesh, settings)?;
        self.navigation = Some(navigation);
        Ok(())
    }
    pub fn replace(&mut self, navigation: Navigation) {
        self.navigation = Some(navigation);
    }
    pub fn get(&self) -> Option<&Navigation> {
        self.navigation.as_ref()
    }
    pub fn get_mut(&mut self) -> Option<&mut Navigation> {
        self.navigation.as_mut()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn headless_world_queries_and_updates_navigation_and_preserves_it_on_failure() {
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<Navigation>();
        let mut app = App::new();
        app.add_plugins(RecastNavigationPlugin);
        let mesh = TriangleMesh {
            vertices: vec![[0., 0., 0.], [0., 0., 16.], [16., 0., 16.], [16., 0., 0.]],
            triangles: vec![[0, 1, 2], [0, 2, 3]],
        };
        let mut world = app.world_mut().resource_mut::<NavigationWorld>();
        world.rebuild(&mesh, NavigationSettings::default()).unwrap();
        let nav = world.get_mut().unwrap();
        assert!(
            nav.path([2., 0., 2.], [14., 0., 14.], false)
                .unwrap()
                .unwrap()
                .reaches_goal
        );
        nav.add_or_replace_box(
            1,
            BoxObstacle {
                min: [6., -1., 0.],
                max: [10., 3., 16.],
            },
        )
        .unwrap();
        nav.update_until_up_to_date().unwrap();
        assert!(
            nav.path([2., 0., 2.], [14., 0., 14.], false)
                .unwrap()
                .is_none()
        );
        assert!(
            !nav.path([2., 0., 2.], [14., 0., 14.], true)
                .unwrap()
                .unwrap()
                .reaches_goal
        );
        let revision = nav.revision();
        let mut invalid = mesh;
        invalid.vertices[0][0] = f32::NAN;
        assert!(
            world
                .rebuild(&invalid, NavigationSettings::default())
                .is_err()
        );
        assert_eq!(world.get().unwrap().revision(), revision);
        world.get_mut().unwrap().remove_key(1).unwrap();
        world.get_mut().unwrap().update_until_up_to_date().unwrap();
        assert!(
            world
                .get()
                .unwrap()
                .path([2., 0., 2.], [14., 0., 14.], false)
                .unwrap()
                .unwrap()
                .reaches_goal
        );
    }
}
