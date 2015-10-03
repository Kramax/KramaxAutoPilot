// This version of ReloadableMonoBehaviour is for a release version of your mod
// It disables the special stuff to make your plugin "reloadable".
// It should be copied into your project itself so your released mode
// has no requirement for the plugin reload extension.
#if !DEBUG
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace KramaxReloadExtensions
{
    public class ReloadableMonoBehaviour : MonoBehaviour
    {
        public MonoBehaviour AddComponentToObject(Type type, GameObject aGameObject)
        {
            var result = aGameObject.AddComponent(type) as MonoBehaviour;
            return result;
        }

        public MonoBehaviour AddComponent(Type type)
        {
            return this.AddComponentToObject(type, this.gameObject);
        }
    }
}
#endif