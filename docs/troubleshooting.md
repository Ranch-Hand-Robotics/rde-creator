# Troubleshooting

## Common Issues

### "Language Model API not available"
**Solution:** Install and activate GitHub Copilot Chat extension

### "Build fails with visibility errors"
**Solution:** Check that `*_BUILDING_LIBRARY` macro is defined in CMakeLists.txt

### "Python imports fail"
**Solution:** Ensure Python path includes workspace and check `setup.py` and `setup.cfg`

### "Node doesn't register as component"
**Solution:** Verify `RCLCPP_COMPONENTS_REGISTER_NODE` macro usage

## Getting Help

- Check the [GitHub Issues](https://github.com/Ranch-Hand-Robotics/rde-creator/issues) page
- Review template-specific documentation
- Examine generated code comments for guidance