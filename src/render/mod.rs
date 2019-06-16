use amethyst::{
    ecs::{
        ReadExpect, 
        Resources, 
        SystemData,
    },
    renderer::{
        pass::{DrawPbrTransparentDesc, DrawSkyboxDesc, DrawPbrDesc},
        types::DefaultBackend as Backend,
        GraphCreator,
        GraphBuilder,
        Factory,
        Kind,
        Format,
        SubpassBuilder,
        RenderGroupDesc,
    },
    window::{
        ScreenDimensions,
        Window,
    }
};


#[derive(Default)]
pub struct BoidyGraphBuilder {
    dimensions: Option<ScreenDimensions>,
    is_dirty: bool,
}

impl GraphCreator<Backend> for BoidyGraphBuilder {
    fn rebuild(&mut self, res: &Resources) -> bool {
        // Copied from the amethyst book, as this is currently the one 
        // condition I can consider a rebuild being necessary
        let new_dimensions = res.try_fetch::<ScreenDimensions>();
        use std::ops::Deref;
        if self.dimensions.as_ref() != new_dimensions.as_ref().map(|d| d.deref()) {
            self.is_dirty = true;
            self.dimensions = new_dimensions.map(|d| d.clone());
            // returns false here to wait a while between changes
            // the next time this is checked, it'll return true
            return false;
        } 
        self.is_dirty
    }

    fn builder(&mut self, factory: &mut Factory<Backend>, res: &Resources) -> GraphBuilder<Backend, Resources> {
        use amethyst::renderer::rendy::{
            graph::present::PresentNode,
            hal::command::{ClearDepthStencil, ClearValue},
        };
        self.is_dirty = false;
        
        if let Some(dimensions) = &self.dimensions {
            let window = <ReadExpect<'_, Window>>::fetch(res);
            let window_kind = Kind::D2(
                dimensions.width() as u32, 
                dimensions.height() as u32, 
                1, 
                1
            );
            let surface = factory.create_surface(&window);
            let surface_format = factory.get_surface_format(&surface);
            let mut graph_builder = GraphBuilder::new();
            let color = graph_builder.create_image(
                window_kind,
                1,
                surface_format,
                Some(ClearValue::Color([0.0, 0.0, 0., 1.0].into()))
            );

            let depth = graph_builder.create_image(
                window_kind,
                1,
                Format::D32Sfloat,
                Some(ClearValue::DepthStencil(ClearDepthStencil(1.0, 0)))  
            );

            let pass = graph_builder.add_node(
                SubpassBuilder::new()
                    // Using PBR rather than DrawFlat here
                    .with_group(DrawPbrDesc::skinned().builder())
                    .with_group(DrawSkyboxDesc::new().builder())
                    .with_group(DrawPbrTransparentDesc::skinned().builder())
                    .with_color(color)
                    .with_depth_stencil(depth)
                    .into_pass(),
            );
            let _present = graph_builder.add_node(PresentNode::builder(factory, surface, color).with_dependency(pass));

            graph_builder
        }
        else {
            panic!("No screen dimensions!")
        }
    }
}
